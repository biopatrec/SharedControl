# BioPatRec/MAVLink Connection Layer

## General Requirements
* [Python 2.7](https://www.python.org/download/releases/2.7/)
  * [PyGame](https://www.pygame.org)
  * [SciPy](http://www.scipy.org)
  * [NumPy](http://www.numpy.org)
* [DroneKit](http://python.dronekit.io/develop/installation.html)
* [DroneKit-SITL](http://python.dronekit.io/develop/sitl_setup.html)
* [MAVProxy](https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html)
* [QGroundControl](http://qgroundcontrol.com/)

## Generic Simulator
The `simulator.py` and `playground.py` files contain all the code necessary for setting up a generic vehicle simulator designed around data collection and parameter tuning. They use keyboard input to control the vehicle (WASD, by default), meaning it can either be used via the keyboard or by using BioPatRec to send the keystrokes. If you meet the above prerequisites, you can simply call these files to start using the simulator.

## Setting Up Virtual Environment
The following steps are for simulating the high-level movement of a quadcopter using the included code.

* Open QGroundControl
* Run the following command in a command prompt:

        mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
* Run the following command in another command prompt:

        dronekit-sitl copter
* Run the following command in another command prompt:

        python controller.py

## Using a QuadCopter
`key_controller.py` and `controller.py` contain all the Python code required to communicate with a remote vehicle using the keyboard or BioPatRec, respectively. 
* You may need to modify the values in the `arm` function of each file to suit your particular configuration. See the [parameter list](http://ardupilot.org/copter/docs/parameters.html) for possible options. 
* You may also need to modify the connection string (`MAVLINK_IP` and `MAVLINK_PORT`) to suit how you plan to connect to the vehicle. See the [documentation](http://python.dronekit.io/guide/connecting_vehicle.html) for more details.

If you already have an [AutoPilot board](http://ardupilot.org/copter/docs/common-autopilots.html) and vehicle set up with [ArduPilot](http://ardupilot.org/copter/docs/introduction.html), then you are ready to connect. [Pixhawk](https://docs.px4.io/en) is another MAVLink-compatible firmware, but included code assumes you are using ArduPilot and some steps may be incompatible with other firmwares.

## Adding New Avoidance Systems
All collision avoidance code is located in `avoidance.py`. Any new function (preferably titled `avoidance_<NAME>`, should follow this format:
* Input
  * Relative heading with respect to the front of the vehicle (radians)
  * Velocity in the heading direction (m/s)
  * 8-element list containing distance measurements (m)
    * This list has sector 0 centered at the front of the vehicle
    * Sectors are 45-degree arcs, and increment counter-clockwise
* Output
  * Tuple containing the change in angle (rad/s) and change in velocity (m/s), respectively
After implementing the function, add a new constant to indicate your new method and an `elif` statement under `update_trajectory()` to integrate it.
