# multicopter-nnbs-control
An Implementation of Neural Network Back-Stepping (NNBS) Controller for Multicopter UAVs. 

This project is mainly for the course project and is a re-implementation in PX4-Autopilot of the following Ph.D thesis: 

> 郑晓龙.*几类欠驱动系统的神经网络反步控制*.2020.哈尔滨工业大学,PhD dissertation.

Thanks the author a lot. 

## Milestone

Up to now, this project has been preliminarily realized NNBS-based position control for multicopters in PX4. 

NNBS-based attitude control will be implemented if time permits. 

## How to run? 

The project has been successfully tested under Ubuntu20.04 + ROS Noetic + Gazebo 11.12.0. 

### Build the code

* Create a ROS workspace and clone the code. 

```bash
mkdir -p nnbs_ws/src && cd nnbs_ws/src
catkin_init_workspace
git clone https://github.com/Kindn/multicopter-nnbs-control.git
cd multicopter-nnbs-control
git submodule update --init --recursive
```

* Setup dependencies for PX4 and mavros. 

```bash
./px4/Tools/setup/ubuntu.sh
./mavros/mavros/scripts/install_geographiclib_datasets.sh
```

* Build the workspace. 

```bash
catkin build mavros
catkin build offboard_control
catkin build px4 # Not necessary.
```

## Run SITL simulation & offboard control

* Launch mavros. 

```bash
cd ../../
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557
```

* Launch PX4 SITL simulation.

Open a new terminal in the repository.

```bash
cd px4
make px4_sitl_nnbs gazebo  			# for default world with no wind or
make px4_sitl_nnbs gazebo___windy   # for windy world.
```

The parameters of the wind plugin (e.g. wind velocity) can be set in `px4/Tools/sitl_gazebo/worlds/windy.world`.

* Launch offboard control. 

Open a new terminal in the workspace. 

```bash
source devel/setup.bash
roslaunch offboard_control polynomial.launch output_fpath:="/path/to/data.txt"  # for polynomial trajectory tracking or 
roslaunch offboard_control hover.launch output_fpath:="/path/to/data.txt" recording_time:=20 # for hovering.
```

The offboard control node will export set-point and feed-back data to `/path/to/data.txt`.  

The polynomial node starts to record the data as soon as the vehicle starts to track the trajectory, and stop recording when it outputs `Trajectory finished`.

The hovering node starts to record the data as soon as the distance between the feed-back position and the target position is less than a threshold (0.2m),  and the recording operation will last  `recording_time` second(s). 

## Analyze the result

The python script `data_analyzer.py` is used to visualize the result and compute the RMSE. 

Usage: 

```bash
python3 data_analyzer.py /path/to/data.txt
```
