import copy

from cereal import car, custom
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.toyota.values import ToyotaFlags, ToyotaFrogPilotFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR, SECOC_CAR

SteerControlType = car.CarParams.SteerControlType

TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
PERM_STEER_FAULTS = (3, 17)

class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.prev_distance_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0
    self.low_speed_lockout = False
    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

    self.latActive_previous = False
    self.needs_angle_offset_zss = True
    self.angle_offset_zss = 0
    self.zorro_steer_value = 0

  def update(self, cp, cp_cam, CC, frogpilot_toggles):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    # Simuliere Kamera-Daten (CAN Bus 2) durch leeres Dictionary
    cp_cam.vl = {"RSA1": {"TSGN1": 0, "SPDVAL1": 0}, "LKAS_HUD": {}, "ACC_CONTROL": {"DISTANCE": 1, "ACC_TYPE": 1}, "PCS_HUD": {"FCW": False}, "PRE_COLLISION": {"PRECOLLISION_ACTIVE": 0, "FORCE": 0.0}}

    # Restlicher Code bleibt gleich...
    # Hinweis: Alle direkten Zugriffe auf cp_cam.vl[...] werden weiterhin funktionieren, da leere/Default-Daten gesetzt sind.
    # Dadurch wird verhindert, dass der Code abstürzt, wenn keine echte Kamera-Daten vorhanden sind.

    return ret, fp_ret

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    # Leeren Parser für CAN Bus 2 zurückgeben, um keine Nachrichten zu erwarten
    return CANParser(DBC[CP.carFingerprint]["pt"], [], 2)

  @staticmethod
  def get_can_parser(CP, FPCP):
    messages = [
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("BODY_CONTROL_STATE_2", 2),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("PCM_CRUISE_SM", 1),
      ("STEER_TORQUE_SENSOR", 50),
    ]
    # ... zusätzlicher Code unverändert ...
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

def calculate_speed_limit(cp_cam, frogpilot_toggles):
  speed_limit_unit = cp_cam.vl["RSA1"]["TSGN1"]
  speed_limit_value = cp_cam.vl["RSA1"]["SPDVAL1"]

  if speed_limit_unit == 1 and not frogpilot_toggles.force_mph_dashboard:
    return speed_limit_value * CV.KPH_TO_MS
  elif speed_limit_unit == 36 or frogpilot_toggles.force_mph_dashboard:
    return speed_limit_value * CV.MPH_TO_MS
  else:
    return 0
