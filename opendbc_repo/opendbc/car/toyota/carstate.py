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

    # Gear-shifter wird nicht verwendet, da GEAR_PACKET entfernt wurde.
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)
    self.distance_button = 0
    self.pcm_follow_distance = 0
    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    # BLINKERS_STATE
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    # ESP_CONTROL
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    # PCM_CRUISE_2
    ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
    ret.carFaultedNonCritical = False #cp.vl["PCM_CRUISE_2"]["TEMP_ACC_FAULTED"] != 0
    ret.cruiseState = structs.CarState.CruiseState()
    ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
    ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS

    # PCM_CRUISE
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    ret.cruiseState.nonAdaptive = self.pcm_acc_status in (1, 2, 3, 4, 5, 6)

    # WHEEL_SPEEDS
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * 1.015
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # STEER_ANGLE_SENSOR
    ret.steeringAngleDeg = -(cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"])
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    # STEER_TORQUE_SENSOR
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True
    if self.accurate_steer_angle_seen:
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)
      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x
    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # EPS_STATUS
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS
    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS
      ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    # Nicht verwendete Felder explizit auf False oder None setzen
    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    ret.parkingBrake = False
    ret.brakePressed = False
    ret.brakeHoldActive = False
    ret.gas = 0.
    ret.gasPressed = False
    ret.engineRpm = 0
    ret.gearShifter = 3
    ret.leftBlindspot = False
    ret.rightBlindspot = False
    ret.buttonEvents = []
    ret.genericToggle = False

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Nur für Corolla die gewünschten Nachrichten erlauben
    if CP.carFingerprint == CAR.TOYOTA_COROLLA:
      pt_messages = [
        ("BLINKERS_STATE", 0.15),
        ("ESP_CONTROL", 3),
        ("EPS_STATUS", 25),
        ("WHEEL_SPEEDS", 80),
        ("STEER_ANGLE_SENSOR", 80),
        ("PCM_CRUISE", 33),
        ("STEER_TORQUE_SENSOR", 50),
        ("PCM_CRUISE_2", 33),
      ]
      cam_messages = []
    else:
      # Standardverhalten für andere Fahrzeuge
      pt_messages = [
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
        ("VSC1S07", 20),
        ("ENGINE_RPM", 42),
        ("GEAR_PACKET", 1),
        ("PCM_CRUISE_2", 33),
      ]
      cam_messages = [("LKAS_HUD", 1)]

    ret = {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0)}
    # Für den Corolla keinen Bus.cam anlegen
    if cam_messages:
      ret[Bus.cam] = CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2)
    return ret
