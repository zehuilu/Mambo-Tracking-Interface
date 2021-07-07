# Mambo-Tracking-Interface

This is the Python interface for RTD and the Parrot [Mambo](https://www.parrot.com/us/drones/parrot-mambo-fpv) Quadrotor.


# Dependencies

**Mambo-Tracking-Interface** has been tested on Ubuntu 18.04.4 LTS with Python 3.6.9. You can use the latest version of Python 3 and dependencies. Here is the list of all dependencies this repository uses:
```
Python>=3.6.9
numpy
scipy
transforms3d
matplotlib
pyparrot
owl==5.1.279 # You can't download this by pip
```

Use [pip](https://pip.pypa.io/en/stable/) to install the required packages:
```
$ sudo pip3 install numpy scipy transforms3d matplotlib
$ git clone https://github.com/zehuilu/Mambo-Tracking-Interface.git
```

To install [pyparrot](https://github.com/amymcgovern/pyparrot), you can follow the official documentation [website](https://pyparrot.readthedocs.io/en/latest/installation.html).

This repository utilizes [PhaseSpace](https://www.phasespace.com/) Motion Capture System for the state estimation. Since we are unable to distribute its Python API `owl.py` here, you will have to find `owl.py` and copy to `<MAIN_DIRECTORY>/lib` manually. Feel free to change the state estimation function to utilize your own motion capture system.


# Path Configuration

Assume you are currently under the main directory of this repository, then create a directory named `/traj_csv_files` by
```
$ cd <MAIN_DIRECTORY>
$ mkdir traj_csv_files
$ mkdir sysid_data
```

This directory `/traj_csv_files` stores the desired trajectories csv files generated by quadrotor_RTD in 2 Hz, and the low-level controller reads the latest csv file as the desired trajectory in 10 Hz. When the interface is initializing, it will delete all the files in `/traj_csv_files` and make sure there are only trajectories files under this trajectory when the controller is running.


# Communication with PhaseSpace and MATLAB RTD

This project uses mocap for absolute positions and attitudes. Between mocap and LLC (Low-Level Controller, i.e LLC.py), the communication is via TCP/IP. You need to start mocap first, and then run LLC. If the drone is flying outside the mocap region, you can’t get any new data in TCP/IP and all the scripts will wait for new data. So the callback function get_states_mocap() in helper_function.py will land the drone immediately for the safety. The default host and port are ‘127.0.0.1’ (equivalent to ‘localhost’) and 9000. The communication between MATLAB and LLC is via UDP. LLC provides the positions and velocities of the drone to MATLAB. The default host and port are ‘127.0.0.1’ (equivalent to ‘localhost’) and 11000.

Sometimes you may want to shut down the current mocap.py and LLC.py and rerun them, but you may get errors like “The current port is occupied.” You can either wait for a few minutes or change the port value accordingly. But do not mess up with the different ports.





# How to run this interface

Open Terminal 1, type
```
cd <MAIN_DIRECTORY>
python3 src/mocap_phasespace.py
```

Open Terminal 2, type
```
cd <MAIN_DIRECTORY>
python3 src/run_mambo.py
```

After you see the reminder message in Terminal 2, run the quadrotor_RTD planner via MATLAB.


