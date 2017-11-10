# Project_Titan

## Simulation

#### Start PX4 Simulation:
1. Go to Firmware folder.
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
