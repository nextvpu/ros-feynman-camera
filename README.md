# feynman_camera

A ROS driver for Nextvpu 3D cameras.

## Install

This package supports ROS and tested in Melodic(ubuntu 18.04 lts).

1. Install [ROS](http://wiki.ros.org/ROS/Installation).

2. Install dependences
    ```sh
    sudo apt install libusb-1.0-0-dev ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
    ```

3. Create a [ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)(if you don't have one)
	 ```sh
    mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	source devel/setup.bash
    ```
    or
    ```sh
    mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make install
	source install/setup.bash
    ```
	
4. Pull the repository into your ROS workspace
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/nextvpu/ros-feynman-camera
    ```

5. cp feynman udev rule
    ```sh
    roscd feynman_camera
    sudo cp nextvpu-usb.rules /etc/udev/rules.d
    ```

6. Go to catkin workspace and compile feynman_camera
    ```sh
    cd ~/catkin_ws
    catkin_make --pkg feynman_camera
    ```
    or install:
      ```sh
    cd ~/catkin_ws
    catkin_make install --pkg feynman_camera
    ```
    or clean:
      ```sh
    cd ~/catkin_ws
    catkin_make clean --pkg feynman_camera
    ```

## Run feynman_camera

If you didn't add `source $YOUR_WORKSPACE/devel/setup.bash` to your `.bashrc`, remember to source it when open a new terminal :)

or

If you didn't add `source $YOUR_WORKSPACE/install/setup.bash` to your `.bashrc`, remember to source it when open a new terminal :)

### Examples

#### Set correct device_id of camera

edit launch/feynman.launch and change the device_id to proper value.

#### Use feynman

`roslaunch feynman_camera feynman.launch`

## Important Topics
(* means device_id of camera)  
* `/feynman_camera/*/depth`:depth raw data,sensor_msgs::Image(16bit integer)
* `/feynman_camera/*/depth_raw_left`:depth left raw data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/depth_raw_right`:depth right raw data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/temperature`:cpu and projector temperature,feynman_camera::temp_info(2 float point)
* `/feynman_camera/*/dotcloud`:dot cloud data,sensor_msgs::PointCloud2
* `/feynman_camera/*/rgb`:rgb sensor data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/sensor_raw_left`:left sensor raw data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/sensor_raw_right`:right sensor raw data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/sensor_rectify_left`:rectify left sensor data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/sensor_rectify_right`:rectify right sensor data,sensor_msgs::Image(rgb888)
* `/feynman_camera/*/cnn_info`:cnn bbox info,feynman_camera::cnn_info

## Useful Services

This package provides multiple [ros services](http://wiki.ros.org/Services) for users to get useful information and set up devices. To know more about using these services, please check [this tutorial](http://wiki.ros.org/rosservice).(* means device_id of camera)

* `/feynman_camera/*/cameraparam`:return cameraparam
* `/feynman_camera/*/streammode`:set stream mode
mode 0:sensor raw+rgb
mode 1:sensor raw or sensor rectify
mode 2:depth
mode 2:depth+cnn

* `/feynman_camera/*/depthmode`:set depth mode of denoise/fusion/zoom/stitch
* `/feynman_camera/*/exposure`:set exposure
data: {isauto: 0, leftexposureus: 0, rightexposureus: 0, leftgain: 0, rightgain: 0}
isauto=0:manual exposure
isauto =1:auto exposure
when isauto is equal to 0,you can set left and right sensor's exposure time in microseconds and set gain

* `/feynman_camera/*/setprojector`:set projector on(1) or off(0)
* `/feynman_camera/*/switchrectify`:switch sensor data of rectify('rectify') or not('vi')

### Examples

After launching an feynman camera, you can get camera param by the following command(* means device_id of camera)
1. get camera param
    ```sh
    rosservice call /feynman_camera/*/cameraparam
    ```

2. change stream mode
    ```sh
    rosservice call /feynman_camera/*/streammode 0 #set stream mode to output sensor raw data
    rosservice call /feynman_camera/*/streammode 1 #set stream mode to output sensor raw or rectify data
    rosservice call /feynman_camera/*/streammode 2 #set stream mode to output depth data
    rosservice call /feynman_camera/*/streammode 3 #set stream mode to output depth and cnn data
    ```

The usage of other services,  is same as above examples.

## License

Copyright 2019 NextVPU Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*