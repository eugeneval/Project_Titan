# Project_Titan

## Simulation

#### Start PX4 Simulation:
1. Go to Firmware folder in Terminal.
2. Run `make posix jmavsim` to start in JMAVSim, or
`make posix_sitl_default gazebo` to start in Gazebo.

*__Note:__ these simulations are currently for the default quadcopter and bear no resemblance to Project Titan.*

#### Control
Open QGroundControl. It will automatically connect and can be used to control the quad in the simulation. Alternatively, it can be controlled from the pxh console in the command line that opens when you start the sim.

###### PXH console commands

* `commander takeoff` to take off.
* `commander land` to land
* `commander status` for current system system status
* `shutdown` to shutdown the simulation

## DroneKit

* The PX4 simulation is accessed on UDP port `127.0.0.1:14540`
* Vehicle info is accessed as attributes of the `vehicle` class. Examples:
    * `vehicle.version` for the autopilot firmware version
    * `vehicle.velocity` for velocity
    * `vehicle.battery` for battery status
    * `vehicle.home_location` for the set home location
* PX4 mode changes are not currently fully supported in DroneKit and have their own custom function, `PX4setMode`.
    * *Note: this is currently only capable of switching to AUTO*
* Use `PX4Command(wp, type)` to create a command to navigate to a waypoint `wp`, with `type` taking:
    * `"TO"` for a takeoff command
    * `"WP"` for a navigation to a waypoint
    * `"LND"` for a land command
