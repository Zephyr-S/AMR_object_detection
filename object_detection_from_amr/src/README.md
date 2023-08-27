## applications.robotics.mobile.object-detection
Object Detection is an AI sample application based on OpenVINO that detects objects, identifies, locate them, and publishes to the Robot Operating System (ROS) topic in the end.

## Dependencies

In order to build and run Object_Detection application, Ubuntu 20.04 with ROS Foxy is required. Docker with all packages can be found [here](https://github.com/intel-innersource/applications.robotics.mobile.container/tree/main/amr_ubuntu2004_openvino_sdk_env).

## Docs

[Documentation Page](docs/Design.md)

## Building

Make sure you have Open Model Zoo demos downloaded and compiled

```
source /opt/intel/openvino_2021/bin/setupvars.sh
mkdir /home/eiforamr/ros2_ws/omz
cd /home/eiforamr/ros2_ws/omz
cmake -DCMAKE_BUILD_TYPE=Release /opt/intel/openvino_2021/deployment_tools/open_model_zoo/demos
cmake --build . -j4
```

Within the docker, run:
```
source /opt/ros/foxy/setup.bash
source /home/eiforamr/ros2_ws/install/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
colcon build --packages-select object_detection --cmake-args -DOPENCV_AMR_VER=4.5.5 -DCMAKE_PREFIX_PATH=/home/eiforamr/ros2_ws/omz/intel64/Release/lib/ -DOPEN_ZOO_HEADERS=/opt/intel/openvino_2021/deployment_tools/open_model_zoo/demos/common/cpp/
```

To build without tests, run:
```
colcon build --packages-select object_detection --cmake-args -DBUILD_TESTING=OFF -DOPENCV_AMR_VER=4.5.5 -DCMAKE_PREFIX_PATH=/home/eiforamr/ros2_ws/omz/intel64/Release/lib/ -DOPEN_ZOO_HEADERS=/opt/intel/openvino_2021/deployment_tools/open_model_zoo/demos/common/cpp/
```

## Running

Source the wandering install directory:
```
source <path/to/object_detection_ros_workspace>/install/setup.bash
```
### Running hints
Make sure that your ROS_HOME environment is set properly, in the folder that has enough permissions:
```
mkdir ${HOME}/${ROS_DISTRO}/.ros
export ROS_HOME="${HOME}/${ROS_DISTRO}/.ros"
export ROS_LOG_DIR="${ROS_HOME}/ros_log"
export ROS_WORKSPACE="${ROS_HOME}/ros_ws"
```

In order to run rviz on the host within the same LAN (not to take resources from the robot), make sure you have the same `ROS_DOMAIN_ID` env set on those both machines, as ROS topics can overlap. e.g.:
`export ROS_DOMAIN_ID=2`

## Launching Object Detection node
Terminal 1
To run it on CPU(default)
```
source /opt/ros/foxy/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
source <path/to/object_detection_ros_workspace>/install/setup.bash
ros2 run object_detection object_detection_node
```
To run it on Myriad/GPU:
Use launch file in dir object_detection/launch/standalone_launch_MYRIAD.launch.py to alter the device

Terminal 2:
```
source /opt/ros2_ws/install/setup.bash
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth:=true
```
## Visulaization 
Terminal 3:

```

source /opt/ros/foxy/setup.bash
source <path/to/object_detection_ros_workspace>/install/setup.bash

rviz2
```

rviz2 cofig:
image = /camera/image_tracked
markerarray = /detected_objects_marker_array

## Tests
* [Object Detection tests](object_detection/tests/README.md#testing)
