# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


## Installation

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)

* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

* libpcl >= 1.6
  * Linux: can be installed using `sudo apt install libpcl-dev`
  * Windows: recommend using [libpcl](http://www.pointclouds.org/downloads/windows.html).
  * MAC:
        
		1. install [homebrew](https://brew.sh/)
   
        2. update homebrew 
        	```bash
        	$> brew update
        	```

        3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
        	```bash
        	$> brew tap brewsci/science
        	```

        4. view pcl install options
        	```bash
        	$> brew options pcl
        	```

        5. install PCL 
        	```bash
        	$> brew install pcl
        	```

## Basic Build Instructions

- Clone this repo.
- Ray Mode: `python build.py --clean --run --ray_mode`
- Point Cloud Mode: `python build.py --clean --run --point_cloud_mode`
- Segmentation Mode: `python build.py --clean --run --seg_mode`
- Ransac 2D Mode: `python build.py --clean --run --ransac_2d_mode` [Quiz of RANSAC 2D]
- Ransac 3D Mode: `python build.py --clean --run --ransac_3d_mode` [Quiz of RANSAC 3D]
- PCL Cluster Mode: `python build.py --clean --run --pcl_cluster_mode`
- Self Cluster Mode : `python build.py --clean --run --self_cluster_mode` [Quiz of CLSUTER 2D]
- Self Cluster/RANSAC Mode:`python build.py --clean --run --self_cluster_ransc_mode`
- Pcl Filter Mode: `python build.py --clean --run --pcl_filter_mode`
- Final Project Mode : `python build.py --clean --run --final_project_mode`[Final Project Delivery]
- Final Project Self Mode: `python build.py --clean --run --final_project_self_mode`
  [Final Project Delivery my own implementation for RANSAC and Clustring]
  Note: 
  1. `--data2` can be added to the above command to run data set 2.
  2. if the above project is build once can be used as follow
     `python build.py --run --final_project_self_mode --data1` or
     `python build.py --run --final_project_self_mode --data2`.
     Parametrs can be changed from build.py (no need for build).



## Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

## Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
