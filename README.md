# Mambo-Tracking-Interface

This is a trajectory tracking interface for Parrot [Mambo](https://www.parrot.com/us/drones/parrot-mambo-fpv) Quadrotor via [pyparrot](https://github.com/amymcgovern/pyparrot). By default, **Mambo-Tracking-Interface** is able to obtain state estimation from motion capture systems [PhaseSpace](https://www.phasespace.com/) or [Qualisys](https://www.qualisys.com/), and you're welcome to write functions for your own motion capture system. The purpose of this repository is that prople can foucs on motion planning / path planning / trajectory planning and **Mambo-Tracking-Interface** can take over actual trajectory tracking as long as your planner outputs trajectories as csv files in a specific directory.

* **Mambo-Tracking-Interface** has built-in support for a motion planner called RTD, [Reachability-based Trajectory Design](https://asmedigitalcollection.asme.org/DSCC/proceedings/DSCC2019/59162/V003T19A010/1070634). Usage is shown below. A video of **Mambo-Tracking-Interface** wit RTD is shown [here](https://www.youtube.com/watch?v=1cldHVQK3Yw), starting at 1'32''.


# Note (By Sept. 23, 2021)
I've been testing multiple Mambo, the acceptable latest firmware is v 3.0.25. The latest firmware v 3.0.26 is NOT working with [pyparrot](https://github.com/amymcgovern/pyparrot) sometimes. I recommend not to update your Mambo firmware if you want to use either [pyparrot](https://github.com/amymcgovern/pyparrot) or **Mambo-Tracking-Interface**.


# Dependencies

This repo has been tested with:
* Ubuntu 18.04.4 LTS, Python 3.6.9
* Ubuntu 20.04.3 LTS, Python 3.8.10
* macOS 11.4, Python 3.9.6

You can use the latest version of Python 3 and dependencies. Note that [pyparrot](https://github.com/amymcgovern/pyparrot) does not support Python 2. Here is the list of all dependencies this repository uses:
```
Python>=3.6.9
numpy
scipy
transforms3d
matplotlib
pyparrot
qtm  # for motion capture system Qualisys
owl==5.1.279  # for motion capture system PhaseSpace, you can't download it by pip
```

* Use [pip](https://pip.pypa.io/en/stable/) to install the required packages:
```
$ sudo pip3 install numpy scipy transforms3d matplotlib
$ git clone https://github.com/zehuilu/Mambo-Tracking-Interface.git
```

* To install [pyparrot](https://github.com/amymcgovern/pyparrot), you can follow the official documentation [website](https://pyparrot.readthedocs.io/en/latest/installation.html).

* This repository supports [PhaseSpace](https://www.phasespace.com/) Motion Capture System for the state estimation. Since we are unable to distribute its Python API `owl.py` here, you have to manually download `owl.py` to `<MAIN_DIRECTORY>/lib`. I am not sure if the motion capture system codes [run_mocap_phasespace.py](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/scripts/run_mocap_phasespace.py) work perfectly when using `owl.py` of any later version than 5.1.279. But in the worst case scenario, you only need to do slight changes in [run_mocap_phasespace.py](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/scripts/run_mocap_phasespace.py) to adapt the new API evocation manner.

* This repository supports [Qualisys](https://www.qualisys.com/) Motion Capture System for the state estimation. See more examples about how to use [Qualisys Python SDK](https://github.com/qualisys/qualisys_python_sdk) in [How-to-Use-Qualisys-Motion-Capture-System](https://github.com/zehuilu/How-to-Use-Qualisys-Motion-Capture-System).
To install [Qualisys Python SDK](https://github.com/qualisys/qualisys_python_sdk),
```
$ sudo pip3 install qtm
```

Feel free to change the state estimation [publisher](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/scripts/run_mocap_phasespace.py) and [callback function](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/lib/MamboControllerInterface.py#L232-L302) to adapt your own motion capture system.


# Directory Initialization

Assume you are currently under the main directory of this repository, then create two directories by
```
$ cd <MAIN_DIRECTORY>
$ mkdir traj_csv_files
$ mkdir sysid_data
```

* The directory `/traj_csv_files` stores the desired trajectories (csv files) generated by an arbitrary planner, and **Mambo-Tracking-Interface** reads the latest csv file as the desired trajectory in 10 Hz.

* The directory `/sysid_data` stores the actual flight data (csv files) for debugging, visualization, and system identification.


# Communication between motion capture system and Mambo controller

**Mambo-Tracking-Interface** utilizes motion capture system to obstain state estimation (positions and attitudes). **Mambo-Tracking-Interface** utilizes TCP/IP communication to connect [motion capture system](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/scripts/run_mocap_phasespace.py) with [Mambo controller](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/scripts/run_mambo.py). When Mambo controller cannot receive data from the motion capture system, it will automatically land for safety.

In some cases, a function call like time.sleep(), matplotlib.pyplot.pause(), or [asyncio](https://pypi.org/project/asyncio/) functions can make data accumulated in a communication channel. This is caused by the nature of TCP/IP and UDP protocols. Therefore, you are unable to get latest data whenever you call the callback function. A solution is to use a customized UDP protocol so that it cleans up the channel/queue and puts the latest data in a new channel/queue whenever you call the callback function. Deatils are shown [here](https://stackoverflow.com/questions/62648835/avoid-accumulation-of-data-in-udp-socket-or-read-newest-data-from-udp-socket). Examples are in [Tutorial-About-TCP-IP-and-UDP-Communications](https://github.com/zehuilu/Tutorial-About-TCP-IP-and-UDP-Communications).

A customized UDP protocol is [`lib/UdpProtocol.py`](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/lib/UdpProtocol.py). To run the motion capture system with this customized UDP protocol,
```
$ cd <MAIN_DIRECTORY>
$ python3 scripts_aimslab/run_mocap_qualisys_for_planner.py
```


# How a trajectory csv file looks like?
A trajectory csv file is a 2D array, and you can read it by `numpy.genfromtxt()`. See more details in [`lib/csv_helper.py`](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/lib/csv_helper.py#L34-L58).


An example is shown below (all units are SI Units, i.e., second, meter, meter/second, etc.):

[[ t0, t1, t2 ... ]

[  px0, px1, px2, ... ]

[  py0, py1, py2, ... ]

[  pz0, pz1, pz2, ... ]

[  vx0, vx1, vx2, ... ]

[  vy0, vy1, vy2, ... ]

[  vz0, vz1, vz2, ... ]]


# How to run this interface

1. Run the motion capture system PhaseSpace by
```
$ cd <MAIN_DIRECTORY>
$ python3 scripts/run_mocap_phasespace.py
```

* Or run the motion capture system Qualisys by
```
cd <MAIN_DIRECTORY>
python3 scripts/run_mocap_qualisys.py
```


2. Run Mambo controller
```
cd <MAIN_DIRECTORY>
python3 scripts/run_mambo.py
```


3. After you run Mambo controller, Mambo will take off and be hovering if there is no trajectory files in `/traj_csv_files`. You can either start you planner after you launch Mambo controller or before.


3. To run [RTD](https://asmedigitalcollection.asme.org/DSCC/proceedings/DSCC2019/59162/V003T19A010/1070634), Reachability-based Trajectory Design, as the motion planner.

Follow step 1 and 2 of this section. After you see the reminder in Mambo controller, run quadrotor_RTD planner via MATLAB.


# How to tune the controller
1. Set `FLAG_DELETE_CSV_BEGIN` as 0 in `configuration.json` such that the Mambo controller won't delete existing trajectories when it's initialized.
```
"FLAG_DELETE_CSV_BEGIN": 0
```

2. Generate a trajectory by running [scripts/generate_traj_for_tuning.py](https://github.com/zehuilu/Mambo-Tracking-Interface/blob/master/scripts/generate_traj_for_tuning.py). You can pick different parameters to generate different trajectories.
```
$ python3 scripts/generate_traj_for_tuning.py
```

3. Run the motion capture system and the Mambo controller
```
cd <MAIN_DIRECTORY>
python3 scripts/run_mocap_qualisys.py
```

```
cd <MAIN_DIRECTORY>
python3 scripts/run_mambo.py
```

4. Observe data visualizations of desired and actual trajectory, revise controller parameters in `configuration.json` and execute the trajectory again.


# Others
Feel free to post your questions in [Issues](https://github.com/zehuilu/Mambo-Tracking-Interface/issues) or [Discussions](https://github.com/zehuilu/Mambo-Tracking-Interface/discussions). **Mambo-Tracking-Interface** is still under my maintenance as of August 27, 2021.


I will add a section to explain configuration files (json files) soon. (edited in August 27, 2021)

