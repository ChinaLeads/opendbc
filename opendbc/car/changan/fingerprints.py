from cereal import car
from opendbc.car.changan.values import CAR

Ecu = car.CarParams.Ecu

# China branch targets a single vehicle (Changan Z6 iDD) and uses opendbc's
# official fixed-fingerprint mechanism: launch_env.sh exports
# 'FINGERPRINT="CHANGAN_Z6_IDD"' which maps to FingerprintSource.fixed in
# opendbc/car/car_helpers.py. No FW fingerprint data is required for that path.
#
# Real FW strings from an actual Z6 iDD have NOT been captured yet. Once they
# are collected (e.g. via Fingerprinting 2.0 on a real car), they can be added
# here to enable automatic FW fingerprinting. Until then keep these entries
# empty to avoid matching unrelated vehicles.
FW_VERSIONS = {
  CAR.CHANGAN_Z6: {},
  CAR.CHANGAN_Z6_IDD: {},
}