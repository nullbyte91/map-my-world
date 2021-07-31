## Map-my-world
This project we will create a 2D occupancy grid and 3D octomap from a simulated environment using our own robot and sensors with the RTAB-Map package.

### rtabmap_ros
RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Most importantly, the quality of the documentation on ROS Wiki (http://wiki.ros.org/rtabmap_ros) is very high.

The recommended robot configuration requires:

1. A 2D Laser, providing sensor_msgs/LaserScan messages
2. Odometry sensors, providing nav_msgs/Odometry messages
3. 3D Camera, compatible with openni_launch, openni2_launch or freenect_launch ROS packages

### Demo Videos and Images
| 👉 [Map-my-world](https://youtu.be/PF4Z7xRsVOM) |
| ------------------------------------------------------------ |
| [![Map-my-world Demo](https://github.com/nullbyte91/map-my-world/blob/master/images/HLD_View.png)](https://youtu.be/PF4Z7xRsVOM) |

#### Gazebo View
![Gazbo view](https://github.com/nullbyte91/map-my-world/blob/master/images/gazebo_view.png)

#### 2D Map
![2D Map](https://github.com/nullbyte91/map-my-world/blob/master/images/2d_map_update.png)

#### 3D Map
![3D Map](https://github.com/nullbyte91/map-my-world/blob/master/images/3d_map.png)

#### Occupancy Grid Map
![Occupancy Grid Map](https://github.com/nullbyte91/map-my-world/blob/master/images/occupancy_grid_map.png)

#### Detected Feature
![Detected Feature](https://github.com/nullbyte91/map-my-world/blob/master/images/Detected_Feature.png)

### 🗃 Project structure
```python
├── images
├── my_robot
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── __MACOSX
│   ├── launch
│   │   ├── amcl.launch
│   │   ├── localization.launch
│   │   ├── mapping.launch
│   │   ├── robot_description.launch
│   │   └── world.launch
│   ├── maps
│   │   ├── my_robot.pgm
│   │   ├── my_robot.yaml
│   │   ├── rtabmap.pgm
│   │   └── rtabmap.yaml
│   ├── meshes
│   │   └── hokuyo.dae
│   ├── model
│   │   ├── aws_robomaker_warehouse_TrashCanC_01
│   │   │   ├── materials
│   │   │   │   └── textures
│   │   │   │       └── aws_robomaker_warehouse_TrashCanC_01.png
│   │   │   ├── meshes
│   │   │   │   ├── aws_robomaker_warehouse_TrashCanC_01_collision.DAE
│   │   │   │   └── aws_robomaker_warehouse_TrashCanC_01_visual.DAE
│   │   │   ├── model.config
│   │   │   └── model.sdf
│   │   ├── aws_robomaker_warehouse_WallB_01
│   │   │   ├── materials
│   │   │   │   └── textures
│   │   │   │       └── aws_robomaker_warehouse_WallB_01.png
│   │   │   ├── meshes
│   │   │   │   ├── aws_robomaker_warehouse_WallB_01_collision.DAE
│   │   │   │   └── aws_robomaker_warehouse_WallB_01_visual.DAE
│   │   │   ├── model.config
│   │   │   └── model.sdf
│   │   └── ball
│   │       ├── model.config
│   │       └── model.sdf
│   ├── package.xml
│   ├── rviz
│   │   └── my_robot_config.rviz
│   ├── urdf
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── worlds
│       ├── empty.world
│       ├── no_roof_small_warehouse.world
│       ├── test_world.world
│       ├── UdacityOffice_withball.world
│       └── UdacityOffice.world
└── README.md

```
### 🖖 Quick Start
```bash
mkdir -p catkin_ws/src && pushd catkin_ws/src
git clone https://github.com/nullbyte91/map-my-world.git
cd ..

# Build package
catkin_make

# Terminal 1
source devel/setup.bash
export GAZEBO_MODEL_PATH=`rospack find my_robot`/model/:$GAZEBO_MODEL_PATH
roslaunch my_robot world.launch

# Terminal 2
source devel/setup.bash
roslaunch my_robot mapping.launch

# teleop_twist_keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# rtabmap viewer
rtabmap-databaseViewer ~/.ros/rtabmap.db

```

### Reference
[aws warehouse Model](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)

[rtabmap_ros](http://wiki.ros.org/rtabmap_ros)
