# ekf_state_estimator

## 1. 简介

这是一个ROS功能包，其中复现了报告中基于9DIMU的姿态估计算法，包含两个节点：

* **orientation_estimator**: 订阅MAVROS转发的来自PX4的传感器原始数据，使用EKF算法估计无人机姿态并发布。
* **data_converter**: 将orientation_estimator估计的姿态和MAVROS发布的姿态估计分别转换为欧拉角后再发布，并且提供保存数据的服务。

scripts目录下的plot.py用于画出data_converter保存的数据。

## 2. 运行

1. 平台：Ubuntu （推荐版本20.04）+ PX4-AutoPilot（推荐版本1.11） + ROS（推荐版本noetic）+ MAVROS + QGC + Gazebo（推荐版本11）+ Eigen3。可以参考PX4 User Guide中的指导来配置，配置到可以成功运行PX4/Gazebo仿真和MAVROS，并能用QGC控制仿真中的飞机即可。

2. 启动qgc、MAVROS和仿真。

3. 将本功能包放入ROS工作空间中编译。

4. 启动本功能包中的节点：

   ```bash
   roslaunch ekf_state_estimation orientation_estimator.launch
   ```

5. 此时可用QGC控制仿真中的飞机飞行。

6. 保存数据

   ```bash
   rosservice call /data_converter/save_data /path/to/data/file
   ```

7. 画出结果。在scripts目录下执行

   ```
   python3 plot.py /data_converter/save_data /path/to/data/file
   ```

   