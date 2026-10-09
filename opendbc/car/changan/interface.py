from opendbc.car.changan.carcontroller import CarController
from opendbc.car.changan.carstate import CarState
from opendbc.car.changan.radar_interface import RadarInterface
from opendbc.car.changan.values import CarControllerParams, ChanganSafetyFlags
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw,
                  alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "changan"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.changan)]
    ret.safetyConfigs[0].safetyParam |= ChanganSafetyFlags.LONGITUDINAL.value

    ret.steerActuatorDelay = 0.12
    ret.steerLimitTimer = 1
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    # Z6 publishes front-radar/ACC object frames on the camera (bus 2) CAN
    # stream.  Keep the radar path enabled; RadarInterface applies its own
    # validity/continuity gates.
    ret.radarUnavailable = False
    ret.minSteerSpeed = 0
    ret.minEnableSpeed = -1
    ret.pcmCruise = False
    ret.openpilotLongitudinalControl = True
    ret.autoResumeSng = True
    ret.stopAccel = -0.5
    ret.longitudinalActuatorDelay = 0.3

    # Fields that upstream moved into the deprecated group.  They still drive
    # the starting/stopping state machine and the BSM display, so set them
    # through the group accessor.
    ret.deprecated.enableBsm = True
    ret.deprecated.radarTimeStep = 0.10
    ret.deprecated.vEgoStopping = 0.25
    ret.deprecated.vEgoStarting = 0.25
    ret.deprecated.stoppingDecelRate = 0.3
    ret.deprecated.startingState = True
    ret.deprecated.startAccel = 1.2

    return ret
