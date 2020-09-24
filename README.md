# Vision-Based Navigation in ROS

## Overview
This repo contains source code for vision-based navigation in ROS. We combine deep learning and traditional computer vision methods along with ArUco markers to obtain relative positioning between the camera and marker. Both monocular and stereo vision is tested for comparison. The figure below shows the high-level architecture. 


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
    ./install_opencv4.1.1_Jetson.sh.
 

NB! There has been some issues when combining ROS melodic and OpenCV <= 4.x.x, so it may be safe to install OpenCV <= 3.4.x. We installed 3.4.3 by simply changing 4.1.1 with 3.4.3 everywhere in the sh file (install_opencv4.1.1_Jetson.sh).

### Create catkin workspace

Now, create a catkin workspace and include our ROS package as well as ROS package for bridging opencv and ROS (vision_opencv):

    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/oysteinvolden/vision-based-navigation.git
    git clone https://github.com/ros-perception/vision_opencv.git 
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release

Building in release mode makes sure you maximize performance. 


## Hardware

This section shows the specific hardware in use. 

  - 2 x Camera: [FLIR Blackfly S GigE (BFS-PGE-13Y3C-C)](https://www.edmundoptics.com/p/bfs-pge-13y3c-c-poe-gige-blackflyr-s-color-camera/40198/).
  - 2 x Lens: [Edmund Optics](https://www.edmundoptics.com/p/35mm-fl-wide-angle-low-distortion-lens/23288/). 
  - GPIO connector: [Hirose HR10](https://www.flir.co.uk/products/hirose-hr10-6-pin-circular-connector/).
  - Lidar: [Ouster Os1](https://ouster.com/products/os1-lidar-sensor/).
  - PC: [Nvidia Jetson Xavier](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit).
  - DC/DC converters.
  - PoE adapters.
  
  ![Hardware overview](doc/figures/hardware_design.png)

## Basic usage

### Camera driver
We use a ROS compatible [camera driver](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver). By this, the camera driver and the object detection pipeline can interchange data via ROS topics. Follow the instructions in this github repository to create a catkin workspace. We use hardware triggering (GPIO connector) for stereo setup as described under "Multicamera Master-Slave Setup" in [github repo](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver). When GPIO cables are connected correctly, do the following:

- Change camera ids to serial numbers of the actual cameras in params/stereo_camera_example.yaml.
- Make sure left camera is master camera (primary).
- in launch/acquisition.launch, change from test_params.yaml to stereo_camera_example.yaml (line 22). 


To launch the driver, open a terminal and type:

    cd ~/spinnaker_ws
    source devel/setup.bash
    roslaunch spinnaker_sdk_camera_driver acquisition.launch
    


### LiDAR driver
We use a ROS compatible [LiDAR driver](https://github.com/ouster-lidar/ouster_example/tree/master/ouster_ros) for verification of camera measurements, i.e. not a part of the core functionality. 


**Credit: The pipeline is further extended and developed for 3D localization tasks based on relevant object detection frameworks such as [YOLO ROS: Real-Time Object Detection for ROS](https://github.com/leggedrobotics/darknet_ros) by Marko Bjelonic and [YOLO ROS: Real-Time Object Detection for ROS](https://github.com/pushkalkatara/darknet_ros) by Pushkal Katara.**



