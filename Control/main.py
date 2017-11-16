
print("Starting")
from dronekit import connect, VehicleMode
from pymavlink import mavutil
# from PX4 import PX4setMode, PX4Command
# from navigation import get_location_offset_meters
from modules.PX4 import PX4setMode, PX4Command
from modules.navigation import get_location_offset_meters, get_location_metres, get_distance_metres
import time

def send_ned_position(pos_x, pos_y, pos_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    vehicle.mode = VehicleMode("OFFBOARD")

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask
        pos_x, pos_y, pos_z, # x, y, z positions
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)



    vehicle.send_mavlink(msg)

# Settings
connection_string = '127.0.0.1:14540'
MAV_MODE_AUTO = 4

# Connect to vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
time.sleep(1)
vehicle.wait_ready('autopilot_version')

def arm_and_takeoff(targetAlt):
    wp = get_location_offset_meters(home, 0, 0, targetAlt)
    cmds.add(PX4Command(wp, "TO"))
    cmds.upload()
    time.sleep(1)

    vehicle.mode = VehicleMode("MISSION")
    time.sleep(1)
    print("Vehicle mode should be MISSION: %s" % vehicle.mode.name)
    vehicle.armed = True
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=targetAlt*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

def goto(pos_x, pos_y, pos_z=vehicle.location.global_relative_frame.alt):

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, pos_x, pos_y)
    targetDistance = get_distance_metres(currentLocation, targetLocation)

    send_ned_position(pos_x, pos_y, pos_z)
    vehicle.mode = VehicleMode("OFFBOARD")
    print("Vehicle mode should be OFFBOARD: %s" % vehicle.mode.name)

    while True:
        send_ned_position(pos_x, pos_y, 0)
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        if remainingDistance<=targetDistance*0.01:
            print("Arrived at target")
            break
        print "Distance to target: ", remainingDistance
        time.sleep(0.1)

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
# print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
# print(" Local Location: %s" % vehicle.location.local_frame)
print(" Attitude: %s" % vehicle.attitude)
print(" Velocity: %s" % vehicle.velocity)
print(" GPS: %s" % vehicle.gps_0)
# print(" Gimbal status: %s" % vehicle.gimbal)
print(" Battery: %s" % vehicle.battery)
# print(" EKF OK?: %s" % vehicle.ekf_ok)
# print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
# print(" Rangefinder: %s" % vehicle.rangefinder)
# print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
# print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
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
home = vehicle.location.global_relative_frame

# Load commands
cmds = vehicle.commands
cmds.clear()

arm_and_takeoff(10)

goto(10, 0)
goto(0, 10)
goto(-10, 0)
goto(0, -10)

vehicle.mode = VehicleMode("RTL")
time.sleep(1)
print("Vehicle mode should be RTL: %s" % vehicle.mode.name)
while vehicle.armed == True:
    time.sleep(3)
    print("Waiting for landing...")

# shutdown = False;
# missionNum = 0;
# while shutdown == False:
#
#     consoleCommand = raw_input("Enter a command: ")
#     if consoleCommand == "takeoff":
#         wp = get_location_offset_meters(home, 0, 0, 10)
#         cmds.add(PX4Command(wp, "TO"))
#     if consoleCommand == "north":
#         wp = get_location_offset_meters(wp, 10, 0, 0)
#         cmds.add(PX4Command(wp, "WP"))
#     if consoleCommand == "east":
#         wp = get_location_offset_meters(wp, 0, 10, 0)
#         cmds.add(PX4Command(wp, "WP"))
#     if consoleCommand == "south":
#         wp = get_location_offset_meters(wp, -10, 0, 0)
#         cmds.add(PX4Command(wp, "WP"))
#     if consoleCommand == "west":
#         wp = get_location_offset_meters(wp, 0, -10, 0)
#         cmds.add(PX4Command(wp, "WP"))
#     if consoleCommand == "land":
#         wp = get_location_offset_meters(home, 0, 0, 0)
#         cmds.add(PX4Command(wp, "LND"))
#     if consoleCommand == "shutdown":
#         shutdown = True
#         vehicle.armed = False
#         time.sleep(1)
#         vehicle.close()
#         exit("\nCompleted")

    # cmds.upload()
    # time.sleep(1)
    # vehicle.armed = True
    #
    # while vehicle.commands.next > 0:
    #     cmds.download()
    #     cmds.wait_ready()
    #     time.sleep(1)
    #     print("Waiting for mission compeletion...Remaining: %s" % vehicle.commands.next)
    #
    # print("Mission completed")
    # vehicle.armed = False
    # cmds.clear()
    # cmds.upload()



# Disarm vehicle
vehicle.armed = False
time.sleep(1)


vehicle.close()
print("\nCompleted")
