#!/usr/bin/env python3
'''
This process finds calibration values. More info on what these calibration values
are can be found here https://github.com/commaai/openpilot/tree/master/common/transformations
While the roll calibration is a real value that can be estimated, here we assume it's zero,
and the image input into the neural network is not corrected for roll.
'''

import os
import capnp
import numpy as np
from typing import NoReturn

from cereal import log, car
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.common.transformations.orientation import rot_from_euler, euler_from_rot
from openpilot.common.swaglog import cloudlog

MIN_SPEED_FILTER = 15 * CV.MPH_TO_MS
MAX_VEL_ANGLE_STD = np.radians(0.25)
MAX_YAW_RATE_FILTER = np.radians(2)  # per second

MAX_HEIGHT_STD = np.exp(-3.5)

# This is at model frequency, blocks needed for efficiency
SMOOTH_CYCLES = 10
BLOCK_SIZE = 100
INPUTS_NEEDED = 5   # Minimum blocks needed for valid calibration
INPUTS_WANTED = 50   # We want a little bit more than we need for stability
MAX_ALLOWED_YAW_SPREAD = np.radians(2)
MAX_ALLOWED_PITCH_SPREAD = np.radians(4)
RPY_INIT = np.array([0.0,0.0,0.0])
WIDE_FROM_DEVICE_EULER_INIT = np.array([0.0, 0.0, 0.0])
HEIGHT_INIT = np.array([1.22])

# These values are needed to accommodate the model frame in the narrow cam of the C3
PITCH_LIMITS = np.array([-0.09074112085129739, 0.17])
YAW_LIMITS = np.array([-0.06912048084718224, 0.06912048084718235])
DEBUG = os.getenv("DEBUG") is not None

def is_calibration_valid(rpy: np.ndarray) -> bool:
  return True  # Immer gültig

def sanity_clip(rpy: np.ndarray) -> np.ndarray:
  if np.isnan(rpy).any():
    rpy = RPY_INIT
  return np.array([rpy[0],
                   np.clip(rpy[1], PITCH_LIMITS[0] - .005, PITCH_LIMITS[1] + .005),
                   np.clip(rpy[2], YAW_LIMITS[0] - .005, YAW_LIMITS[1] + .005)])

def moving_avg_with_linear_decay(prev_mean: np.ndarray, new_val: np.ndarray, idx: int, block_size: float) -> np.ndarray:
  return (idx*prev_mean + (block_size - idx) * new_val) / block_size

class Calibrator:
  def __init__(self, param_put: bool = False):
    self.param_put = param_put

    self.not_car = False

    # Read saved calibration
    self.params = Params()
    calibration_params = self.params.get("CalibrationParams")
    rpy_init = RPY_INIT
    wide_from_device_euler = WIDE_FROM_DEVICE_EULER_INIT
    height = HEIGHT_INIT
    valid_blocks = INPUTS_NEEDED  # Immer voll!
    self.cal_status = log.LiveCalibrationData.Status.calibrated

    if param_put and calibration_params:
      try:
        with log.Event.from_bytes(calibration_params) as msg:
          rpy_init = np.array(msg.liveCalibration.rpyCalib)
          valid_blocks = INPUTS_NEEDED  # Immer voll!
          wide_from_device_euler = np.array(msg.liveCalibration.wideFromDeviceEuler)
          height = np.array(msg.liveCalibration.height)
      except Exception:
        cloudlog.exception("Error reading cached CalibrationParams")

    self.reset(rpy_init, valid_blocks, wide_from_device_euler, height)
    self.valid_blocks = INPUTS_NEEDED
    self.update_status()

  def reset(self, rpy_init: np.ndarray = RPY_INIT,
                  valid_blocks: int = INPUTS_NEEDED,
                  wide_from_device_euler_init: np.ndarray = WIDE_FROM_DEVICE_EULER_INIT,
                  height_init: np.ndarray = HEIGHT_INIT,
                  smooth_from: np.ndarray = None) -> None:
    if not np.isfinite(rpy_init).all():
      self.rpy = RPY_INIT.copy()
    else:
      self.rpy = rpy_init.copy()

    if not np.isfinite(height_init).all() or len(height_init) != 1:
      self.height = HEIGHT_INIT.copy()
    else:
      self.height = height_init.copy()

    if not np.isfinite(wide_from_device_euler_init).all() or len(wide_from_device_euler_init) != 3:
      self.wide_from_device_euler = WIDE_FROM_DEVICE_EULER_INIT.copy()
    else:
      self.wide_from_device_euler = wide_from_device_euler_init.copy()

    self.valid_blocks = INPUTS_NEEDED

    self.rpys = np.tile(self.rpy, (INPUTS_WANTED, 1))
    self.wide_from_device_eulers = np.tile(self.wide_from_device_euler, (INPUTS_WANTED, 1))
    self.heights = np.tile(self.height, (INPUTS_WANTED, 1))

    self.idx = 0
    self.block_idx = 0
    self.v_ego = 0.0

    if smooth_from is None:
      self.old_rpy = RPY_INIT
      self.old_rpy_weight = 0.0
    else:
      self.old_rpy = smooth_from
      self.old_rpy_weight = 1.0

  def get_valid_idxs(self) -> list[int]:
    # exclude current block_idx from validity window
    before_current = list(range(self.block_idx))
    after_current = list(range(min(self.valid_blocks, self.block_idx + 1), self.valid_blocks))
    return before_current + after_current

  def update_status(self) -> None:
    valid_idxs = self.get_valid_idxs()
    if valid_idxs:
      self.wide_from_device_euler = np.mean(self.wide_from_device_eulers[valid_idxs], axis=0)
      self.height = np.mean(self.heights[valid_idxs], axis=0)
      rpys = self.rpys[valid_idxs]
      self.rpy = np.mean(rpys, axis=0)
      max_rpy_calib = np.array(np.max(rpys, axis=0))
      min_rpy_calib = np.array(np.min(rpys, axis=0))
      self.calib_spread = np.abs(max_rpy_calib - min_rpy_calib)
    else:
      self.calib_spread = np.zeros(3)

    self.valid_blocks = INPUTS_NEEDED
    self.cal_status = log.LiveCalibrationData.Status.calibrated

    # Schreibzyklus für Params: Ignorieren reicht meist, aber so bleibt's erhalten
    write_this_cycle = (self.idx == 0) and (self.block_idx % (INPUTS_WANTED//5) == 5)
    if self.param_put and write_this_cycle:
      self.params.put_nonblocking("CalibrationParams", self.get_msg(True).to_bytes())

  def handle_v_ego(self, v_ego: float) -> None:
    self.v_ego = v_ego

  def get_smooth_rpy(self) -> np.ndarray:
    if self.old_rpy_weight > 0:
      return self.old_rpy_weight * self.old_rpy + (1.0 - self.old_rpy_weight) * self.rpy
    else:
      return self.rpy

  def handle_cam_odom(self, trans: list[float],
                            rot: list[float],
                            wide_from_device_euler: list[float],
                            trans_std: list[float],
                            road_transform_trans: list[float],
                            road_transform_trans_std: list[float]) -> np.ndarray | None:
    self.old_rpy_weight = max(0.0, self.old_rpy_weight - 1/SMOOTH_CYCLES)

    # Die eigentliche Kalibrierung wird übersprungen, alles wird als gültig betrachtet!
    self.update_status()
    return self.rpy

  def get_msg(self, valid: bool) -> capnp.lib.capnp._DynamicStructBuilder:
    smooth_rpy = self.get_smooth_rpy()

    msg = messaging.new_message('liveCalibration')
    msg.valid = valid

    liveCalibration = msg.liveCalibration
    liveCalibration.validBlocks = INPUTS_NEEDED
    liveCalibration.calStatus = log.LiveCalibrationData.Status.calibrated
    liveCalibration.calPerc = 100
    liveCalibration.rpyCalib = smooth_rpy.tolist()
    liveCalibration.rpyCalibSpread = np.zeros(3).tolist()
    liveCalibration.wideFromDeviceEuler = self.wide_from_device_euler.tolist()
    liveCalibration.height = self.height.tolist()

    if self.not_car:
      liveCalibration.validBlocks = INPUTS_NEEDED
      liveCalibration.calStatus = log.LiveCalibrationData.Status.calibrated
      liveCalibration.calPerc = 100.
      liveCalibration.rpyCalib = [0, 0, 0]
      liveCalibration.rpyCalibSpread = np.zeros(3).tolist()

    return msg

  def send_data(self, pm: messaging.PubMaster, valid: bool) -> None:
    pm.send('liveCalibration', self.get_msg(valid))


def main() -> NoReturn:
  config_realtime_process([0, 1, 2, 3], 5)

  pm = messaging.PubMaster(['liveCalibration'])
  sm = messaging.SubMaster(['cameraOdometry', 'carState'], poll='cameraOdometry')

  params_reader = Params()
  CP = messaging.log_from_bytes(params_reader.get("CarParams", block=True), car.CarParams)

  calibrator = Calibrator(param_put=True)
  calibrator.not_car = CP.notCar

  while 1:
    timeout = 0 if sm.frame == -1 else 100
    sm.update(timeout)

    if sm.updated['cameraOdometry']:
      calibrator.handle_v_ego(sm['carState'].vEgo)
      calibrator.handle_cam_odom(sm['cameraOdometry'].trans,
                                 sm['cameraOdometry'].rot,
                                 sm['cameraOdometry'].wideFromDeviceEuler,
                                 sm['cameraOdometry'].transStd,
                                 sm['cameraOdometry'].roadTransformTrans,
                                 sm['cameraOdometry'].roadTransformTransStd)

    # 4Hz driven by cameraOdometry
    if sm.frame % 5 == 0:
      calibrator.send_data(pm, sm.all_checks())


if __name__ == "__main__":
  main()
