"""Live monitoring (P4): a virtual drone speaking MAVLink, and DroneGuard watching it."""

import os

# pymavlink's connections speak MAVLink 1 unless told otherwise, and MAVLink 1
# cannot carry ESC_TELEMETRY_1_TO_4 (message id 11030 > 255). This must be set
# before pymavlink.mavutil is first imported.
os.environ.setdefault("MAVLINK20", "1")
