from cereal import car
from opendbc.car.changan.values import CAR

Ecu = car.CarParams.Ecu

# Firmware versions are intentionally left empty.
# The China branch uses fixed-fingerprinting via the FINGERPRINT
# environment variable (set in launch_env.sh).  Real factory strings
# must be collected on the target vehicle with openpilot's
# Fingerprinting 2.0 flow and added here before automatic recognition
# can be enabled.
FW_VERSIONS = {
  CAR.CHANGAN_Z6_IDD: {},
}
