
<h3 align="center">
    <p style="font-size: 70px;">ARIO Dataset Tools</p>
</h3>
<div align="center">

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/ario-dataset/ARIO-dataset-agilex/blob/master/LICENSE)
![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)
![ROS1](https://img.shields.io/badge/ROS-noetic-blue.svg)

</div>

---

## Introduction

These are ARIO dataset tools. Data collection, data saving, and data publishing codes are included. The data is saved in a fixed format. For details on the format, see Directory structure translation.

## Testing environment

![build](https://img.shields.io/badge/build-catkin_make-blue.svg)
[![PCL](https://img.shields.io/badge/PCL-1.10.0-blue.svg)](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.10.0)
[![rapidyaml](https://img.shields.io/badge/rapidyaml-v0.5.0-blue.svg)](https://github.com/biojppm/rapidyaml/tree/b35ccb150282760cf5c2d316895cb86bd161ac89)
[![rapidjson](https://img.shields.io/badge/rapidjson-v1.1.0-blue.svg)](https://github.com/Tencent/rapidjson/tree/f54b0e47a08782a6131cc3d60f94d038fa6e0a51)

## Hardware support list

Tested successfully on Agilex [Cobot Magic](https://github.com/agilexrobotics/cobot_magic) robot.

## Installation

Download our source code:

```shell
git clone https://github.com/ario-dataset/ario-tools.git

cd ario-tools

git submodule update --init --recursive
```

## Build

```shell
cd ario-tools

catkin_make
```

## Required sensors & data types

hardware:

- Depth camera with RGB x 3
- Six-degree-of-freedom robotic arm with gripper x 4
- agilex tracer wheel encoder

sensors type:

- `sensor_msgs::Image`
- `sensor_msgs::PointCloud2`
- `sensor_msgs::JointState`
- `sensor_msgs::CameraInfo`
- `geometry_msgs::PoseStamped`
- `nav_msgs::Odometry`

topic name:

- `/camera_l/color/image_raw`
- `/camera_f/color/image_raw`
- `/camera_r/color/image_raw`
- `/camera_l/depth/image_raw`
- `/camera_f/depth/image_raw`
- `/camera_r/depth/image_raw`
- `/master/joint_left`
- `/master/joint_right`
- `/puppet/joint_left`
- `/puppet/joint_right`
- `/master/end_left`
- `/master/end_right`
- `/puppet/end_left`
- `/puppet/end_right`

## Package file documentation

```shell
# package name : collect_data

src/
└── collect_data
    ├── CMakeLists.txt
    ├── config
    │   ├── data_params.yaml
    │   └── information.yaml
    ├── include
    │   ├── blockingDeque.h
    │   ├── dataUtility.h
    │   └── getCurrentPath.h
    ├── launch
    │   ├── run_data_capture.launch # Run the data collection launch
    │   ├── run_data_publish.launch
    │   └── run_data_sync.launch # Synchronize the datasets
    ├── package.xml
    ├── scripts
    │   ├── data_to_hdf5_example.py # Convert raw data to HDF5 format
    │   └── load_data_example.py # load data from an HDF5 file for training
    ├── src
    │   ├── dataCapture.cpp
    │   ├── dataPublish.cpp
    │   ├── dataSync.cpp
    │   └── getCurrentPath.cpp
    └── Thirdparty # git submodule
        ├── rapidjson
        └── rapidyaml
```

## Run

Run the data collection code.

```shell
# Activate environment variables.
source source devel/setup.sh
# Assume it is dataset group 0. You need to specify the path where it is located. The path must already exist; if it does not, an error will be reported.
mkdir /workspace/task0
roslaunch collect_data run_data_capture.launch episodeIndex:=0 datasetDir:=/workspace/task0
```

If successful, the following status will be displayed.

```shell
path: /workspace/task0/episode0
total time: 3.00223
topic: frame in 1 second / total frame
/camera_l/color/image_raw: 20 / 65
/camera_f/color/image_raw: 20 / 62
/camera_r/color/image_raw: 20 / 61
/camera_l/depth/image_raw: 19 / 60
/camera_f/depth/image_raw: 21 / 60
/camera_r/depth/image_raw: 23 / 63
/camera_l/depth/color/points: 21 / 59
/camera_f/depth/color/points: 18 / 54
/camera_r/depth/color/points: 22 / 65
/master/joint_left: 87 / 248
/master/joint_right: 84 / 238
/puppet/joint_left: 83 / 248
/puppet/joint_right: 89 / 243
/master/end_left: 88 / 245
/master/end_right: 84 / 233
/puppet/end_left: 83 / 241
/puppet/end_right: 93 / 250
sum total frame: 2495

config topic: total frame
/camera_l/color/camera_info: 0
/camera_f/color/camera_info: 0
/camera_r/color/camera_info: 0
/camera_l/depth/camera_info: 1
/camera_f/depth/camera_info: 1
/camera_r/depth/camera_info: 1
/camera_l/depth/camera_info: 1
/camera_f/depth/camera_info: 1
/camera_r/depth/camera_info: 1

press ENTER to stop capture:
```

To stop data collection, press ENTER to stop. Writing to the file takes time, so you can try waiting or pressing ENTER several times.

If "Done" is displayed, it means the data just collected has been written to the file.

```shell
Done
[collect_data_dataCapture-1] process has finished cleanly
log file: /home/noetic/.ros/log/21114750-1995-11ef-b6f1-578b5ce9ba2e/collect_data_dataCapture-1*.log
all processes on machine have died, roslaunch will exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

**Note: If there is no data in the topic, the terminal output will be stuck, you need to use Ctrl+C to exit.**

## Directory structure translation

For example, dataset number 0 collected:

```shell
episode0
.
├── arm
│   ├── endPose
│   │   ├── masterLeft
│   │   │   └── 1714373556.360405.json
│   │   ├── masterRight
│   │   │   └── 1714373556.360135.json
│   │   ├── puppetLeft
│   │   │   └── 1714373556.393465.json
│   │   └── puppetRight
│   │       └── 1714373556.386106.json
│   └── jointState
│       ├── masterLeft
│       │   └── 1714373556.360460.json
│       ├── masterRight
│       │   └── 1714373556.360205.json
│       ├── puppetLeft
│       │   └── 1714373556.363036.json
│       └── puppetRight
│           └── 1714373556.363178.json
├── camera
│   ├── color
│   │   ├── front
│   │   │   ├── 1714373556.409885.png
│   │   │   └── config.json
│   │   ├── left
│   │   │   ├── 1714373556.370113.png
│   │   │   └── config.json
│   │   └── right
│   │       ├── 1714373556.358616.png
│   │       └── config.json
│   ├── depth
│   │   ├── front
│   │   │   ├── 1714373556.388914.png
│   │   │   └── config.json
│   │   ├── left
│   │   │   ├── 1714373556.353924.png
│   │   │   └── config.json
│   │   └── right
│   │       ├── 1714373556.376457.png
│   │       └── config.json
│   └── pointCloud
│       ├── front
│       │   ├── 1714373556.248885.pcd
│       │   └── config.json
│       ├── left
│       │   ├── 1714373556.247312.pcd
│       │   └── config.json
│       └── right
│           ├── 1714373556.273297.pcd
│           └── config.json
├── instructions.json
└── robotBase
    └── vel
```

[Example data](https://huggingface.co/datasets/agilexrobotics/ARIO)

## Synchronize the datasets

Run the following code, specifying the path of the task and the index of the `episodeIndex`. When the dataset index is `-1`, it means that all datasets in the task path will be synchronized.

```shell
roslaunch data_tools run_data_sync.launch datasetDir:=/workspace/task0 episodeIndex:=-1
```

After execution, a `sync.txt` file will be generated in the path of each specific dataset. For example, the image data synchronization index file path: `task0/episode0/camera/color/left/sync.txt`.

## Convert raw data to HDF5 format

Run the following code to generate a data.hdf5 file in the path of the task0 task, which includes all datasets (episode0, episode1, ..., episodeX). This file contains the synchronized joint information and the synchronized index file of the image data.

```shell
python3 data_to_hdf5_example.py --datasetDir /workspace/task0
```

## How to load data from an HDF5 file for training

Here's an example of loading files. The paths in the program need to be manually specified as absolute paths. This code snippet does not have any actual functionality; it's just an example program. You can use this example to integrate into your own training code.

```shell
python3 load_data_example.py
```

## How to add sensors tailored to your needs

## Licence

The source code is released under [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0.html) license.

## Contributions

[Agilex robotics](https://github.com/agilexrobotics)

## TODO list

|TODO|state|
|---|---|
|Add support for more hardware devices, such as 2D LiDAR and 3D LiDAR.|❌|
|Add support for tactile and force control messages.|❌|
|Add a web UI to support visualizing data collection, reducing the difficulty of collecting data angles.|❌|

## Contact us

![alt text](.assert/wxQRCode_6-18.png)
