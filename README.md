# RTD_Mambo_interface

This is the Python interface for RTD and the Parrot [Mambo](https://www.parrot.com/us/drones/parrot-mambo-fpv) Quadrotor.

**RTD_Mambo_interface** has been tested on Ubuntu 18.04.4 LTS with Python 3.6.9. Your configuration should satisfy all the version requirements of dependencies.

**1.Dependencies**
This documentation doesn’t mention the Python Standard Libraries here, but the scripts show all the libraries used. I recommend pip for installing all the dependencies.

Here are the resources for pyparrot:
https://github.com/amymcgovern/pyparrot
https://pyparrot.readthedocs.io/en/latest/installation.html
You can follow the documentation to install pyparrot.

**RTD_Mambo_interface** utilizes [PhaseSpace](https://www.phasespace.com/) Motion Capture System for the state estimation. Since we are unable to distribute its Python API named owl.py here, you will have to access it by yourselves. Feel free to change the state estimation function to use your own motion capture system and interface.

numpy (1.17.0)
scipy (1.3.0)
transforms3d (0.3.1)
matplotlib (3.1.1)
pyparrot (1.5.20)
owl (5.1.279)


**2. Path Configuration**
The path for helper_function.py and owl.py are different from the path for the scripts for running the Mambo and mocap. To import the mocap API and helper functions, you need to add their full path first by sys.path.append('/<your_path>/Mambo_scripts/lib').

And don’t delete the folders sysid_data and traj_lib.


**3. Communication with PhaseSpace and MATLAB RTD**
This project uses mocap for absolute positions and attitudes. Between mocap and LLC (Low-Level Controller, i.e LLC.py), the communication is via TCP/IP. You need to start mocap first, and then run LLC. If the drone is flying outside the mocap region, you can’t get any new data in TCP/IP and all the scripts will wait for new data. So the callback function get_states_mocap() in helper_function.py will land the drone immediately for the safety. The default host and port are ‘127.0.0.1’ (equivalent to ‘localhost’) and 9000. The communication between MATLAB and LLC is via UDP. LLC provides the positions and velocities of the drone to MATLAB. The default host and port are ‘127.0.0.1’ (equivalent to ‘localhost’) and 11000.

Sometimes you may want to shut down the current mocap.py and LLC.py and rerun them, but you may get errors like “The current port is occupied.” You can either wait for a few minutes or change the port value accordingly. But do not mess up with the different ports.








**To-do**

sysid data

traj_lib

Desired yaw

How to run the whole stuff
