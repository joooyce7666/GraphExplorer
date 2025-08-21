# GraphExplorer
A Graph-Based Method for Efficient Frontier Detection and Traversability Assessment in Autonomous Exploration

机器平台使用的是冰达品牌的二轮差速机器人，此项目使用了冰达机器人平台的两个功能包，其功能包链接分别为：https://gitee.com/bingda-robot/nanorobot_description
https://gitee.com/bingda-robot/robot_navigation

可以执行以下指令来下载功能包
```bash
cd ~/catkin_ws/src
git clone https://gitee.com/bingda-robot/nanorobot_description.git
git clone https://gitee.com/bingda-robot/robot_navigation.git
git clone https://github.com/joooyce7666/GraphExplorer.git
```

ROS版本为Noetic，可以执行以下指令来安装依赖
```bash
sudo apt-get update
sudo apt-get install ros-noetic-teleop-twist-keyboard ros-noetic-amcl ros-noetic-move-base ros-noetic-slam-gmapping ros-noetic-slam-karto ros-noetic-dwa-local-planner ros-noetic-teb-local-planner ros-noetic-uvc-camera ros-noetic-map-server ros-noetic-hector-slam* ros-noetic-global-planner ros-noetic-navfn
```

安装完依赖后可以执行以下命令来编译

```bash
cd ~/catkin_ws
catkin_make
```

编译成功后，注意在环境变量中加上对应的变量值，打开~/.bashrc文件，在最后加上如下内容
```bash
export BASE_TYPE=NanoRobot
export CAMERA_TYPE=usb70
export LIDAR_TYPE=ydlidar
```

记得使用如下命令刷新一下环境变量
```bash
source ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

打开robot_navigation/launch/robot_slam_laser.launch文件，将open_rviz和simulation和改为true，将<include file="$(find robot_simulation)/launch/simulation_one_robot.launch"/>这一行改为<include file="$(find nanorobot_description)/launch/simulation.launch"/>

然后打开nanorobot_description/launch/simulation.launch，在<arg name="world_name" value="$(find nanorobot_description)/worlds/room.world"/>中写想要做实验的场景的.world包路径，现在可以开始运行仿真环境和rviz了
```bash
cd ~/catkin_ws
source /devel/setup.bash
roslaunch robot_navigation robot_slam_laser.launch
```
再打开一个终端，执行以下命令
```bash
cd ~/catkin_ws
source /devel/setup.bash
roslaunch graph_exploration graph_exploration.launch
```
现在车应该动起来了，如果想要拓扑图可视化，可以在rviz中添加一下marker_array，选择话题，就能看到拓扑图了，保存rviz配置就能让下次自动打开拓扑图的可视化。

graph_exploration.launch的各个参数的意义将在论文发表后详细补充




