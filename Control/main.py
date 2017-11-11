
print("Starting")
from dronekit import connect
# from pymavlink import mavutil
from PX4 import PX4setMode, PX4Command
from navigation import get_location_offset_meters
import time

# Settings
connection_string = '127.0.0.1:14540'
MAV_MODE_AUTO = 4

# Connect to vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
vehicle.wait_ready('autopilot_version')

# Get all vehicle attributes (state), uncomment as needed
print("\nGet all vehicle attribute values:")
print(" Autopilot Firmware version: %s" % vehicle.version)
# print("   Major version number: %s" % vehicle.version.major)
# print("   Minor version number: %s" % vehicle.version.minor)
# print("   Patch version number: %s" % vehicle.version.patch)
# print("   Release type: %s" % vehicle.version.release_type())
# print("   Release version: %s" % vehicle.version.release_version())
# print("   Stable release?: %s" % vehicle.version.is_stable())
# print(" Autopilot capabilities")
# print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
# print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
# print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
# print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
# print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
# print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
# print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
# print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
# print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
# print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
# print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
# print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
# print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
# print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
print(" Global Location: %s" % vehicle.location.global_frame)
print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print(" Local Location: %s" % vehicle.location.local_frame)
print(" Attitude: %s" % vehicle.attitude)
print(" Velocity: %s" % vehicle.velocity)
print(" GPS: %s" % vehicle.gps_0)
print(" Gimbal status: %s" % vehicle.gimbal)
print(" Battery: %s" % vehicle.battery)
print(" EKF OK?: %s" % vehicle.ekf_ok)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Rangefinder: %s" % vehicle.rangefinder)
print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print(" Heading: %s" % vehicle.heading)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
print(" Airspeed: %s" % vehicle.airspeed)    # settable
print(" Mode: %s" % vehicle.mode.name)    # settable
print(" Armed: %s" % vehicle.armed)    # settable

# Waits for a home position to be set before displaying it
home_position_set = False
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)
print(" Home location: %s" % vehicle.home_location)

# Set to Auto mode
PX4setMode(vehicle, MAV_MODE_AUTO)
time.sleep(1)
print("Vehicle mode should be AUTO: %s" % vehicle.mode.name)

# Load commands
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame

# takeoff to 10 meters
wp = get_location_offset_meters(home, 0, 0, 10);
cmds.add(PX4Command(wp, "TO"))

# move 10 meters north
wp = get_location_offset_meters(wp, 10, 0, 0);
cmds.add(PX4Command(wp, "WP"))

# move 10 meters east
wp = get_location_offset_meters(wp, 0, 10, 0);
cmds.add(PX4Command(wp, "WP"))

# move 10 meters south
wp = get_location_offset_meters(wp, -10, 0, 0);
cmds.add(PX4Command(wp, "WP"))

# move 10 meters west
wp = get_location_offset_meters(wp, 0, -10, 0);
cmds.add(PX4Command(wp, "WP"))

# land
wp = get_location_offset_meters(home, 0, 0, 10);
cmds.add(PX4Command(wp, "LND"))

# Upload mission
cmds.upload()
time.sleep(2)

# Arm vehicle
vehicle.armed = True

# monitor mission execution
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print "Moving to waypoint %s" % display_seq
        nextwaypoint = vehicle.commands.next
    time.sleep(1)

# wait for the vehicle to land
while vehicle.commands.next > 0:
    time.sleep(1)


# Disarm vehicle
vehicle.armed = False
time.sleep(1)


vehicle.close()
print("\nCompleted")
