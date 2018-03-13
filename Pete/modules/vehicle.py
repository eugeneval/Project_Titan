from dronekit import VehicleMode, LocationLocal, Command
from dronekit import Vehicle as dronekit_Vehicle
from pymavlink import mavutil
from navigation import get_location_offset_meters, get_location_metres_local, get_distance_metres_local
import time

class Vehicle(dronekit_Vehicle):
    def __init__(self, handler):
        super(Vehicle, self).__init__(handler)

    def arm_and_takeoff(self, targetAlt, accuracy=0.5):
        """Overrides DroneKit command. Takeoff to a target altitude, then loiter."""

        wp = get_location_offset_meters(self.home_location, 0, 0, targetAlt)
        self.commands.add(PX4Command(wp, "TO"))
        self.commands.upload()
        time.sleep(1)

        self.mode = VehicleMode("MISSION")
        time.sleep(1)
        print("Vehicle mode should be MISSION: %s" % self.mode.name)
        self.armed = True
        while True:
            print " Altitude: ", self.location.global_relative_frame.alt
            if self.location.global_relative_frame.alt>=targetAlt-accuracy:
                print "Reached target altitude"
                self.mode = VehicleMode("LOITER")
                time.sleep(0.5)
                break
            time.sleep(1)

    def goto_absolute(self, pos_x, pos_y, pos_z, accuracy=0.5, wait=True, text=True):
        """Go to a position relative to the home position"""

        targetLocation = LocationLocal(pos_x, pos_y, -pos_z)

        self.__send_ned_position(pos_x, pos_y, -pos_z)
        self.mode = VehicleMode("OFFBOARD")
        if text: print("Vehicle mode should be OFFBOARD: %s" % self.mode.name)

        while True:
            self.__send_ned_position(pos_x, pos_y, -pos_z)
            remainingDistance = get_distance_metres_local(self.location.local_frame, targetLocation)
            if remainingDistance<=accuracy:
                if text: print("Arrived at target")
                break
            if wait == False: break
            if text: print "Distance to target: ", remainingDistance
            time.sleep(0.1)

    def goto_relative(self, pos_x, pos_y, pos_z, accuracy=0.5, wait=True, text=True):
        """Go to a position relative to the current posotion"""

        currentLocation = self.location.local_frame
        targetLocation = get_location_metres_local(currentLocation, pos_x, pos_y, -pos_z)\

        self.__send_ned_position(targetLocation.north, targetLocation.east, targetLocation.down)
        self.mode = VehicleMode("OFFBOARD")
        if text: print("Vehicle mode should be OFFBOARD: %s" % self.mode.name)

        while True:
            self.__send_ned_position(targetLocation.north, targetLocation.east, targetLocation.down)
            remainingDistance = get_distance_metres_local(self.location.local_frame, targetLocation)
            if remainingDistance<=accuracy:
                if text: print("Arrived at target")
                break
            if wait == False: break
            if text: print "Distance to target: ", remainingDistance
            time.sleep(0.1)

    def setMaxXYSpeed(self, speed):
        """Required due to DroneKit's vehicle.groundspeed not working with PX4"""

        self.parameters['MPC_XY_VEL_MAX']=speed
        print("Set max speed to: %s" % self.parameters['MPC_XY_VEL_MAX'])
        time.sleep(0.5)

    def returnToLand(self):
        """RTL and wait for auto disarm on land"""

        self.mode = VehicleMode("RTL")
        time.sleep(1)
        print("Vehicle mode should be RTL: %s" % self.mode.name)
        while self.armed == True:
            print("Waiting for landing...")
            time.sleep(3)


    def __send_ned_position(self, pos_x, pos_y, pos_z):
        """Custom message for sending an OFFBOARD position."""

        msg = self.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask
            pos_x, pos_y, pos_z, # x, y, z positions
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.send_mavlink(msg)

    # def wait_for_home(self):
    #     home_position_set = False
    #     @self.on_message('HOME_POSITION')
    #     def listener(self, name, home_position):
    #         global home_position_set
    #         home_position_set = True
    #     while not home_position_set:
    #         print "Waiting for home position..."
    #         time.sleep(1)
    #     print(" Home location: %s" % self.vehicle.home_location)

def PX4setMode(vehicle, mavMode):
    """For PX4 mode switching as it is currently unsuported in DroneKit"""
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)


def PX4Command(wp, type):
    """Builds a command based on a LocationGlobal waypoint and a type (takeoff, waypoint, or land)"""
    if type == "TO":
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    if type == "WP":
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    if type == "LND":
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    return cmd
