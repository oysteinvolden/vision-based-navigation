# Vision-Based Navigation in ROS

## Overview
This repo contains source code for vision-based navigation in ROS. We combine deep learning and traditional computer vision methods along with ArUco markers to obtain relative positioning between the camera and marker. Both monocular and stereo vision is used for comparison. It is tested with Ubuntu 18.04 LTS and ROS melodic on arm-based 64-bit architecture (Nvidia Jetson Xavier). The figure below shows the high-level architecture with focus on stereo vision. 


![Pipeline overview](doc/figures/overview2.png)


Please include the following reference when you use the toolbox:
Ø. Volden (2020)."Vision-Based Navigation in ROS". URL: https://github.com/oysteinvolden/vision-based-navigation

Bibtex entry:
@misc{VoldenVISIONROS2020,
  title = {Vision-{B}ased {N}avigation in {ROS}},
  author = {{{\O}}. Volden},
  howpublished = {\url{https://github.com/oysteinvolden/vision-based-navigation}},
  year = {2020}
}

## Installation

### Dependencies

**CUDA**

Cuda is preinstalled on Nvidia Jetson Xavier.

**ROS**

- If you use a standard PC with x86 architecture, follow the instructions here: http://wiki.ros.org/melodic/Installation/Ubuntu. Full-desktop version is recommended if disk-space is not critical.

- For arm-based Nvidia Jetson Xavier:

      git clone https://github.com/jetsonhacks/installROSXavier
      cd installROSXavier
      ./installROS.sh -p ros-melodic-desktop-full
      
This repo also provide a quick solution to setup a catkin workspace by running the command:

      ./setupCatkinWorkspace.sh
	

**OpenCV**

- For standard laptops, installation instructions from here is recommended: https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/.

- For Jetson Xavier:

      git clone https://github.com/AastaNV/JEP
      cd script
      ./install_opencv4.1.1_Jetson.sh
 

NB! There has been some issues when combining ROS melodic and OpenCV <= 4.x.x, so it may be safe to install OpenCV <= 3.4.x. We installed 3.4.3 by simply changing 4.1.1 with 3.4.3 everywhere in the sh file (install_opencv4.1.1_Jetson.sh).

### Install and configure vision-based navigation pipeline

Create a catkin workspace and include our ROS package as well as ROS package for bridging opencv and ROS (vision_opencv).

    mkdir -p darknet_ws/src
    cd darknet_ws/src
    git clone --recursive https://github.com/oysteinvolden/vision-based-navigation.git
    git clone https://github.com/ros-perception/vision_opencv.git 
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release
    
Repeat to create a second ROS node (one per camera).
    
    mkdir -p darknet_ws2/src
    cd darknet_ws2/src
    git clone --recursive https://github.com/oysteinvolden/vision-based-navigation.git
    git clone https://github.com/ros-perception/vision_opencv.git 
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release
    
NB! To be able to run two darknet_ros nodes in parallell, some modificaions for the second darknet_ros node is neccessary.

- Go into darknet_ros/launch/darknet_ros.launch and
	- change "darknet_ros" to "darknet_ros2" at line 16 and 17 (ns=darknet_ros2)
	- change "darknet_ros" to "darknet_ros2" at line 20. Only change name, keep pkg and type the same.

- Go into darknet_ros/config/ros.yaml and
	- change topic name for detection image from /darknet_ros/detection_image to /darknet_ros/detection_image2

- Go into darknet_ros/src/YoloRosTracker.cpp and
	- change from "ros::NodeHandle nh" to "ros::NodeHandle nh2" (Constructor)
	- change "nodeHandle(nh)" to "nodeHandle(nh2)" (next line in the code) 

- Go into darknet_ros/include/darknet_ros/YoloRosTracker.hpp and
	- change from "explicit YoloRosTracker(ros::NodeHandle nh)" to "explicit YoloRosTracker(ros::NodeHandle nh2)".
 
 
**Configuration**

It is assumed that the trained weights and configuration (cfg) file is available for YoloV3 (darknet). 

- darknet folder:
	- Adjust Makefile such that GPU, Opencv, CuDNN, CuDNN_HALF and OpenMP is enabled (uncommented). 
	- Also enable jetson Xavier Arch and disable all others (in Makefile). 

- darknet_ros:
	- Put trained weights and cfg file in the yolo_network_config folder. 
	- Update yolov3-spp.yaml in the darknet_ros/config folder so they match those used in the yolo_network_config folder. 
	- Update ros.yaml in the darknet_ros/config folder to subscribe for correct topic: The topic under "camera_reading" should match the ros topic name sent from the camera driver, e.g. /camera_array/cam0/image_raw or /camera_array/cam1/image_raw. Just one camera topic name per ROS node. 
	- Check that darknet_ros.launch and yolov3-spp.launch in the launch folder includes correct weights/cfg and yaml file (yolov3-spp.yaml). 
	
NB! Calibration parameters and camera resolution is defined in the source code. 


### Install and configure camera driver
We use a ROS compatible [camera driver](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver). By this, the camera driver and the object detection pipeline can interchange data via ROS topics. Follow the instructions in this github repository to create a catkin workspace. We use hardware triggering (GPIO connector) for stereo setup as described under "Multicamera Master-Slave Setup" in the [github repo](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver). When GPIO cables are connected correctly, do the following:

- Change camera ids to serial numbers of the actual cameras in params/stereo_camera_example.yaml.
- Make sure left camera is master camera (primary).
- in launch/acquisition.launch, change from test_params.yaml to stereo_camera_example.yaml (line 22). 

**Network configuration**

We use persistant IP to maintain a stable connection, i.e. always reachable at the same IP address. We add an IP address on the 192.168.x.x subnet so we reach the cameras:

	sudo ip a a 192.168.11.172 dev eth0

Then, go into bashrc file (gedit ~/.bashrc) and set ROS_IP to 192.168.x.x. 

If neccessary, increase udp buffer limit (approx 1000 MB/s) by typing the following in terminal:

	sudo sysctl -w net.core.rmem_max=1000214400 
	sudo sysctl -w net.core.rmem_default=1000214400 
	
If neccessary, increase usb buffer limit (approx 1000 MB/s) by typing the following in terminal:

	sudo sh -c "echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb" 
	
NB! You can monitor bandwith consumption over a NIC like eth0 with the linux tool iftop.
	
	sudo iftop -i eth0
	

### Install LiDAR driver
We use a ROS compatible [LiDAR driver](https://github.com/ouster-lidar/ouster_example/tree/master/ouster_ros) for verification of camera measurements, i.e. not a part of the core functionality. Follow the instructions in the github repository to interface and create a catkin workspace. 


## Hardware

This section shows the specific hardware in use. 

  - 2 x Camera: [FLIR Blackfly S GigE (BFS-PGE-13Y3C-C)](https://www.edmundoptics.com/p/bfs-pge-13y3c-c-poe-gige-blackflyr-s-color-camera/40198/).
  - 2 x Lens: [Edmund Optics](https://www.edmundoptics.com/p/35mm-fl-wide-angle-low-distortion-lens/23288/). 
  - GPIO connector: [Hirose HR10](https://www.flir.co.uk/products/hirose-hr10-6-pin-circular-connector/).
  - Lidar: [Ouster Os1](https://ouster.com/products/os1-lidar-sensor/).
  - PC: [Nvidia Jetson Xavier](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit).
  - DC/DC converters.
  - PoE adapters
  - Switch / ethernet cables. 
  
  The figure below shows the power and ethernet interface between the hardware components and the On-Board System (OBS) for the USV.
  
  ![Hardware overview](doc/figures/hardware_design.png)

## Basic usage

### Launch camera driver
To launch the driver, open a terminal and type:

    cd ~/spinnaker_ws
    source devel/setup.bash
    roslaunch spinnaker_sdk_camera_driver acquisition.launch

### Launch LiDAR driver
To launch the driver, open a terminal and type:

    cd ~/myworkspace
    source devel/setup.bash
    roslaunch ouster_ros os1.launch os1_hostname:=os1-991837000010.local lidar_mode:=2048x10 os1_udp_dest:=192.168.11.219
 
- <os1_hostname> can be the hostname (os1-991xxxxxxxxx) or IP of the OS1 (serial number).
- <lidar_mode> is one of 512x10, 512x20, 1024x10, 1024x20, or 2048x10 (optical resolution).  
- <udp_data_dest_ip> is the IP to which the sensor should send data (make sure it is on the 192.168.x.x subnet). 

### Launch vision-based navigation pipeline
Open a terminal and type the following:

	cd ~/darknet_ws
	source devel/setup.bash
	roslaunch darknet_ros yolov3-spp.launch
	
Go into a second terminal and type the following:
	
	cd ~/darknet_ws2
	source devel/setup.bash
	roslaunch darknet_ros yolov3-spp.launch
	
Hence, you have one ROS node per camera and they run simultaneously. 
	

### ROS topics

**Images**

* **`/camera_array/cam0/image_raw`** ([sensor_msgs/Image])
* **`/camera_array/cam1/image_raw`** ([sensor_msgs/Image])

Original image data from a pair of FLIR blackfly S cameras. 
 
  
**Point cloud**

* **`/os1_cloud_node/points`** ([sensor_msgs/Pointcloud2])

    Original point cloud data from an Ouster Os1 lidar. 


**Author: [Oystein Volden](https://www.ntnu.no/ansatte/oystv), oystein.volden@ntnu.no**


**Credit: The pipeline is further extended and developed for 3D localization tasks based on relevant object detection frameworks such as [YOLO ROS: Real-Time Object Detection for ROS](https://github.com/leggedrobotics/darknet_ros) by Marko Bjelonic and [YOLO ROS: Real-Time Object Detection for ROS](https://github.com/pushkalkatara/darknet_ros) by Pushkal Katara.**



