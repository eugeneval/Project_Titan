# Acceptable PX4 modes (from mavutil):
#
# "MANUAL"
# "STABILIZED"
# "ACRO"
# "RATTITUDE"
# "ALTCTL"
# "POSCTL"
# "LOITER"
# "MISSION"
# "RTL"
# "LAND"
# "RTGS"
# "FOLLOWME"
# "OFFBOARD"



print("Starting")
from dronekit import connect, VehicleMode
import time

connection_string = '127.0.0.1:14540'
# Connect to vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
time.sleep(1)
vehicle.wait_ready('autopilot_version')

vehicle.mode = VehicleMode("MISSION")
time.sleep(1)
print("Vehicle mode: %s" % vehicle.mode.name)
time.sleep(1)

vehicle.close()
print("Completed")
