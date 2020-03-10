# roboy3
## Prerequisites
- Ubuntu 18
- ROS melodic
```sudo apt install ros-$ROS_DISTRO-desktop-full libeigen3-dev libxml2-dev coinor-libipopt-dev \
qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 \
qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs \
qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings \
ros-$ROS_DISTRO-moveit-msgs doxygen swig mscgen ros-$ROS_DISTRO-grid-map \
ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-aruco-detect \
ros-$ROS_DISTRO-effort-controllers libxml++2.6-dev ros-$ROS_DISTRO-robot-localization libalglib-dev \
ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions \
ros-$ROS_DISTRO-robot-state-publisher
```
- [iDynTree](https://github.com/robotology/iDynTree/tree/b65ad9939152c89dc2f7dc484b6e8687882d6b34) commit b65ad99
```
cd path/to/iDyntree
mkdir build && cd build
cmake ..
make -j9
sudo make install
```
## Build
```mkdir roboy3_ws
cd roboy3_ws
git clone --recursive https://github.com/Roboy/roboy3.git -b missxa
cd src
git submodule init && git submodule update
cd roboy3_ws
catkin_make
```
## Run
```
source roboy3_ws/devel/setup.bash
roslaunch kindyn robot.launch robot_name:=upper_body simulated:=true
rviz
```
In RViz:
- add the CARDSflow panel in RViz
- at the Displays Sidebar set Global Options > Fixed Frame to “world”
- at the Displays Sidebar add the display types “rviz > Marker”, “rviz > InteractiveMarker” and “rviz > TF”
- drag any red cube, the according endeffector should follow the cube (solves IK for the cube pose)

```
# list joint names
rosparam get /joint_names

# publish desired joint angles targets
 rostopic pub /joint_targets sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['head_axis0', 'head_axis1']
position: [0.5, 0.2]
velocity: [0,0]
effort: [0,0]" 

# get robot state
rostopic echo /robot_state

# execute IK for given position
rosservice call /execute_ik "endeffector: 'hand_right'
type: 1
target_frame: 'torso'
pose:
  position:
    x: -0.61
    y: -0.04
    z: 0.26
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
    
# solve IK for a given position without applying joint angles
rosservice call /ik "endeffector: 'hand_right'
type: 1
target_frame: 'torso'
pose:
  position:
    x: -0.61
    y: -0.04
    z: 0.26
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```


