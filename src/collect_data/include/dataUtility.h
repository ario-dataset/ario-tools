/* Copyright 2024 The The Agilex Robotics Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#pragma once
#ifndef _DATA_UTILITY_H_
#define _DATA_UTILITY_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#ifdef _USENOETIC
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.h>
#endif

class DataUtility
{
public:

    ros::NodeHandle nh;

    std::vector<std::string> cameraColorNames;
    std::vector<std::string> cameraDepthNames;
    std::vector<std::string> cameraPointCloudNames;
    std::vector<std::string> armJointStateNames;
    std::vector<std::string> armEndPoseNames;
    std::vector<std::string> robotBaseVelNames;

    std::vector<std::string> cameraColorTopics;
    std::vector<std::string> cameraDepthTopics;
    std::vector<std::string> cameraPointCloudTopics;
    std::vector<std::string> armJointStateTopics;
    std::vector<std::string> armEndPoseTopics;
    std::vector<std::string> robotBaseVelTopics;

    std::vector<std::string> cameraColorConfigTopics;
    std::vector<std::string> cameraDepthConfigTopics;
    std::vector<std::string> cameraPointCloudConfigTopics;
    std::vector<std::string> armJointStateConfigTopics;
    std::vector<std::string> armEndPoseConfigTopics;
    std::vector<std::string> robotBaseVelConfigTopics;

    std::vector<std::string> cameraColorPublishTopics;
    std::vector<std::string> cameraDepthPublishTopics;
    std::vector<std::string> cameraPointCloudPublishTopics;
    std::vector<std::string> armJointStatePublishTopics;
    std::vector<std::string> armEndPosePublishTopics;
    std::vector<std::string> robotBaseVelPublishTopics;

    std::vector<std::string> cameraColorConfigPublishTopics;
    std::vector<std::string> cameraDepthConfigPublishTopics;
    std::vector<std::string> cameraPointCloudConfigPublishTopics;
    std::vector<std::string> armJointStateConfigPublishTopics;
    std::vector<std::string> armEndPoseConfigPublishTopics;
    std::vector<std::string> robotBaseVelConfigPublishTopics;

    std::vector<std::string> cameraColorParentFrames;
    std::vector<std::string> cameraDepthParentFrames;
    std::vector<std::string> cameraPointCloudParentFrames;
    std::vector<std::string> armJointStateParentFrames;
    std::vector<std::string> armEndPoseParentFrames;
    std::vector<std::string> robotBaseVelParentFrames;

    std::vector<std::string> cameraColorDirs;
    std::vector<std::string> cameraDepthDirs;
    std::vector<std::string> cameraPointCloudDirs;
    std::vector<std::string> armJointStateDirs;
    std::vector<std::string> armEndPoseDirs;
    std::vector<std::string> robotBaseVelDirs;
    std::string instructionsDir;

    std::vector<bool> cameraColorToSyncs;
    std::vector<bool> cameraDepthToSyncs;
    std::vector<bool> cameraPointCloudToSyncs;
    std::vector<bool> armJointStateToSyncs;
    std::vector<bool> armEndPoseToSyncs;
    std::vector<bool> robotBaseVelToSyncs;

    std::vector<bool> cameraColorToPublishs;
    std::vector<bool> cameraDepthToPublishs;
    std::vector<bool> cameraPointCloudToPublishs;
    std::vector<bool> armJointStateToPublishs;
    std::vector<bool> armEndPoseToPublishs;
    std::vector<bool> robotBaseVelToPublishs;

    float publishRate;
    int captureFrameNum;

    std::string datasetDir;
    int episodeIndex;

    std::string episodeDir;

    std::string cameraDir;
    std::string armDir;
    std::string robotBaseDir;

    std::string cameraColorDir;
    std::string cameraDepthDir;
    std::string cameraPointCloudDir;

    std::string armJointStateDir;
    std::string armEndPoseDir;

    std::string robotBaseVelDir;

    DataUtility(std::string datasetDirParam, int episodeIndexParam)
    {
        nh.param<std::vector<std::string>>("dataInfo/camera/color/names", cameraColorNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/names", cameraDepthNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/names", cameraPointCloudNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/names", armJointStateNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/names", armEndPoseNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/names", robotBaseVelNames, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/parentFrames", cameraColorParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/parentFrames", cameraDepthParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/parentFrames", cameraPointCloudParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/parentFrames", armJointStateParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/parentFrames", armEndPoseParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/parentFrames", robotBaseVelParentFrames, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/topics", cameraColorTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/topics", cameraDepthTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/topics", cameraPointCloudTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/topics", armJointStateTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/topics", armEndPoseTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/topics", robotBaseVelTopics, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/pubTopics", cameraColorPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/pubTopics", cameraDepthPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/pubTopics", cameraPointCloudPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/pubTopics", armJointStatePublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/pubTopics", armEndPosePublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/pubTopics", robotBaseVelPublishTopics, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/configTopics", cameraColorConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/configTopics", cameraDepthConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/configTopics", cameraPointCloudConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/configTopics", armJointStateConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/configTopics", armEndPoseConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/configTopics", robotBaseVelConfigTopics, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/pubConfigTopics", cameraColorConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/pubConfigTopics", cameraDepthConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/pubConfigTopics", cameraPointCloudConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/pubConfigTopics", armJointStateConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/pubConfigTopics", armEndPoseConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/pubConfigTopics", robotBaseVelConfigPublishTopics, std::vector<std::string>());

        nh.param<std::vector<bool>>("dataInfo/camera/color/toSyncs", cameraColorToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/depth/toSyncs", cameraDepthToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/pointCloud/toSyncs", cameraPointCloudToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/jointState/toSyncs", armJointStateToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/endPose/toSyncs", armEndPoseToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/robotBase/vel/toSyncs", robotBaseVelToSyncs, std::vector<bool>());

        nh.param<std::vector<bool>>("dataInfo/camera/color/toPublishs", cameraColorToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/depth/toPublishs", cameraDepthToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/pointCloud/toPublishs", cameraPointCloudToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/jointState/toPublishs", armJointStateToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/endPose/toPublishs", armEndPoseToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/robotBase/vel/toPublishs", robotBaseVelToPublishs, std::vector<bool>());

        datasetDir = datasetDirParam;
        episodeIndex = episodeIndexParam;

        episodeDir = datasetDir + "/episode" + std::to_string(episodeIndex);
        // episodeDir = episodeDir.replace("//", "/");

        cameraDir = episodeDir + "/camera";
        armDir = episodeDir + "/arm";
        robotBaseDir = episodeDir + "/robotBase";

        cameraColorDir = cameraDir + "/color";
        cameraDepthDir = cameraDir + "/depth";
        cameraPointCloudDir = cameraDir + "/pointCloud";
        armJointStateDir = armDir + "/jointState";
        armEndPoseDir = armDir + "/endPose";
        robotBaseVelDir = robotBaseDir + "/vel";

        instructionsDir = episodeDir + "/instructions.json";

        for(int i = 0; i < cameraColorNames.size(); i++){
            std::string dir = cameraColorDir + "/" + cameraColorNames.at(i);
            cameraColorDirs.push_back(dir);
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            std::string dir = cameraDepthDir + "/" + cameraDepthNames.at(i);
            cameraDepthDirs.push_back(dir);
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            std::string dir = cameraPointCloudDir + "/" + cameraPointCloudNames.at(i);
            cameraPointCloudDirs.push_back(dir);
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            std::string dir = armJointStateDir + "/" + armJointStateNames.at(i);
            armJointStateDirs.push_back(dir);
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            std::string dir = armEndPoseDir + "/" + armEndPoseNames.at(i);
            armEndPoseDirs.push_back(dir);
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            std::string dir = robotBaseVelDir + "/" + robotBaseVelNames.at(i);
            robotBaseVelDirs.push_back(dir);
        }
        
    }
};

#endif
