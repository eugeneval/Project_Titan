
print("Starting")
from dronekit import connect, VehicleMode, LocationLocal
from pymavlink import mavutil
# from PX4 import PX4setMode, PX4Command
# from navigation import get_location_offset_meters
from modules.PX4 import PX4Command
from modules.navigation import get_location_offset_meters, get_location_metres_local, get_distance_metres_local
import time

def send_ned_position(pos_x, pos_y, pos_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """

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

###############################################################################
# SETTINGS
###############################################################################
connection_string = '127.0.0.1:14540'

###############################################################################
# CONNECT TO VEHICLE
###############################################################################
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
time.sleep(1)
vehicle.wait_ready('autopilot_version')

###############################################################################
# FUNCTION DEFINITIONS
# Note: these cannot be declared at the begining as they rely on a vehicle object exisiting
###############################################################################
def arm_and_takeoff(targetAlt, accuracy=0.5):
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
        if vehicle.location.global_relative_frame.alt>=targetAlt-accuracy:
            print "Reached target altitude"
            break
        time.sleep(1)

def goto_absolute(pos_x, pos_y, pos_z, accuracy=0.5):
# Go to a position relative to the home position

    targetLocation = LocationLocal(pos_x, pos_y, -pos_z)

    send_ned_position(pos_x, pos_y, -pos_z)
    vehicle.mode = VehicleMode("OFFBOARD")
    print("Vehicle mode should be OFFBOARD: %s" % vehicle.mode.name)

    while True:
        send_ned_position(pos_x, pos_y, -pos_z)
        remainingDistance = get_distance_metres_local(vehicle.location.local_frame, targetLocation)
        if remainingDistance<=accuracy:
            print("Arrived at target")
            break
        print "Distance to target: ", remainingDistance
        time.sleep(0.1)

def goto_relative(pos_x, pos_y, pos_z, accuracy=0.5):
# Go to a position relative to the current posotion

    currentLocation = vehicle.location.local_frame
    targetLocation = get_location_metres_local(currentLocation, pos_x, pos_y, -pos_z)\

    send_ned_position(targetLocation.north, targetLocation.east, targetLocation.down)
    vehicle.mode = VehicleMode("OFFBOARD")
    print("Vehicle mode should be OFFBOARD: %s" % vehicle.mode.name)

    while True:
        send_ned_position(targetLocation.north, targetLocation.east, targetLocation.down)
        remainingDistance = get_distance_metres_local(vehicle.location.local_frame, targetLocation)
        if remainingDistance<=accuracy:
            print("Arrived at target")
            break
        print "Distance to target: ", remainingDistance
        time.sleep(0.1)

def setMaxXYSpeed(speed):
    vehicle.parameters['MPC_XY_VEL_MAX']=speed
    print("Set max speed to: %s" % vehicle.parameters['MPC_XY_VEL_MAX'])
    time.sleep(0.5)

def returnToLand():
    vehicle.mode = VehicleMode("RTL")
    time.sleep(1)
    print("Vehicle mode should be RTL: %s" % vehicle.mode.name)
    while vehicle.armed == True:
        print("Waiting for landing...")
        time.sleep(3)

###############################################################################
# VEHICLE ATTRIBUTES
# Get all vehicle attributes (state), uncomment as needed
###############################################################################
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

###############################################################################
# MISSION
###############################################################################
arm_and_takeoff(10)

setMaxXYSpeed(10)
goto_absolute(10, 0, 10)
goto_absolute(10, 10, 10)
goto_absolute(0, 10, 10)
goto_absolute(0, 0, 10)

setMaxXYSpeed(2)
goto_relative(10, 0, 0, 0.1)
goto_relative(0, 10, 0, 0.1)
goto_relative(-10, 0, 0, 0.1)
goto_relative(0, -10, 0, 0.1)

returnToLand()


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

# Close and exit
vehicle.close()
print("\nCompleted")
