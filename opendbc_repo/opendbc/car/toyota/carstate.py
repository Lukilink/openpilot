import copy
import numpy as np

from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType

TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
PERM_STEER_FAULTS = (3, 17)

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.distance_button = 0
    self.pcm_follow_distance = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    # BODY_CONTROL_STATE
    ret.doorOpen = any([
      cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], 
      cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
      cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], 
      cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]
    ])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    # BRAKE
    ret.brakePressed = cp.vl["PCM_CRUISE_2"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1

    # GAS
    ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0  # 1 = released, 0 = pressed

    # WHEEL SPEEDS
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = float(np.mean([
      ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr
    ]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * 1.015
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # STEERING
    ret.steeringAngleDeg = -(cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"])
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    # GEAR
    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # BLINKER
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    # STEERING TORQUE
    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # EPS FAULTS
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    # ACC/CRUISE STATE
    ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
    ret.cruiseState = structs.CarState.CruiseState()
    ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
    ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
    cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"] if "PCM_CRUISE_SM" in cp.vl else 0
    if ret.cruiseState.speed != 0:
      # Annahme: metrisch
      ret.cruiseState.speedCluster = cluster_set_speed * CV.KPH_TO_MS

    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)

    # ESP
    ret.genericToggle = False  # Keine AUTO_HIGH_BEAM Signal vorhanden
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    # FOLLOW DISTANCE
    self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("ESP_CONTROL", 3),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("STEER_TORQUE_SENSOR", 50),
      ("GEAR_PACKET", 1),
      ("EPS_STATUS", 25),
      ("PCM_CRUISE_2", 33),
    ]
    # Nur die von dir gelieferten Nachrichten werden abonniert.

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
    }
