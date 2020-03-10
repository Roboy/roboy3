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
- (iDynTree)[https://github.com/robotology/iDynTree/tree/b65ad9939152c89dc2f7dc484b6e8687882d6b34] commit b65ad99
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
