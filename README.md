# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


## Installation

## Other Important Dependencies

### For Linuex & MAC
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)

* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)

* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)

* libpcl >= 1.6
  * Linux: can be installed using `sudo apt install libpcl-dev`
  * MAC:
        
		1. install [homebrew](https://brew.sh/)
   
        1. update homebrew 
        	```bash
        	$> brew update
        	```

        2. add  homebrew science [tap](https://docs.brew.sh/Taps) 
        	```bash
        	$> brew tap brewsci/science
        	```

        3. view pcl install options
        	```bash
        	$> brew options pcl
        	```

        4. install PCL 
        	```bash
        	$> brew install pcl
        	```

### For Windows
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)

* VS >= 2017 and PCL 1.9.1
  * for both : [click here for installation instructions](http://unanancyowen.com/en/pcl191/)

or 

* VS >= 2019 and PCL 1.10.1
  * VS2019 : [click here for installation instructions](https://visualstudio.microsoft.com/vs/)
  * PCL 1.10.1 [click here for installation instructions](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.10.1)

* Setup steps:
1. Click“PCL-1.xx.0-AllInOne-msvc20xx-win64.exe”Install
2. In the second step, select "Add PCL to the System Path for All Users"
3. Choose the default installation folder: "C:\Program Files\PCL 1.xx.0"
4. The following direct default installation, and finally pops out "The file name is too long and the installation 
program cannot modify it" when the prompt box is selected, click OK and continue the installation.
5. Choose the default installation folder: "C:\Program Files\PCL 1.xx.0"
[At this time, the OpenNI2 being installed will automatically switch to the "C:\Program Files\OpenNI2" 
folder for installation. It doesn't matter, it's all normal.]
6. After the installation is complete, add all the contents of the folder obtained after decompressing 
PCL-1.xx.0-pdb-msvc20xx-win64.zip to your PCL installation bin directory, such as: C:\Program Files\PCL 1.xx.0\bin.
This is the end of the installation.
7.Add all (PCL,OpenNI2,3rdParty/VTK,3rdParty/FLANN,3rdParty/Qhull,3rdParty/Qhull,Boost/lib) 
bin folder to enviornment path. 


 

## Basic Build Instructions
- Clone this repo.

### Main Enviorenment Selected Modes
- Create build Folder `mkdir build`, `cd build`
- Ray Mode: `cmake .. -DUSE_RAY=ON`
- Point Cloud Mode: `cmake .. -DUSE_POINT_CLOUD=ON`
- Segmentation Mode: `cmake .. -DUSE_SEG=ON`
- PCL Cluster Mode: `cmake .. -DUSE_PCL_CLUSTER=ON`
- Self Cluster/RANSAC Mode:`cmake .. -DUSE_SELF_CLUSTER_RANSAC=ON`
- Pcl Filter Mode: `cmake .. -DUSE_PCL_FILTER=ON`

### Final Project
- Final Project Mode : `cmake .. -DUSE_FINAL_PROJECT=ON`[Final Project Delivery]
- Final Project Self Mode: `cmake .. -DUSE_FINAL_PROJECT_SELF=ON`
  [Final Project Delivery my own implementation for RANSAC and Clustring]
- Run the project using required PCL configuration and dataset 
  * Linux: `./environment ../src/sensors/data/pcd/data_1,0.15,-20.0,-6.0,-2.0,30.0,7.0,5.0,0.2,50,0.3,10,800`
  * Windows: `environment.exe ../src/sensors/data/pcd/data_1,0.15,-20.0,-6.0,-2.0,30.0,7.0,5.0,0.2,50,0.3,10,800`
  This works with GCC. If VS is used , make environment to be start up project , then put the above line 
  in debugger arguments then run it on VS(start instance debug).
  Enviornment Configuration are explined in data_info.

### Quiz
- Navigate to `cd src/quiz`
- Create build Folder `mkdir build`, `cd build`
- Ransac 2D Mode: `cmake .. -USE_RANSAC_2D=ON` [Quiz of RANSAC 2D]
- Ransac 3D Mode: `cmake .. -DUSE_RANSAC_3D=ON` [Quiz of RANSAC 3D]
- Self Cluster Mode : `cmake .. -DUSE_SELF_CLUSTER=ON` [Quiz of CLSUTER 2D]



## Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

## Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
