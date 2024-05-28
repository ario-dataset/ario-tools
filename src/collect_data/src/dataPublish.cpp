/* Copyright 2024 The Agilex Robotics Inc. All Rights Reserved.

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

#include "dataUtility.h"
#include <mutex>
#include <math.h>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <semaphore.h>
#include <iostream>
#include "jsoncpp/json/json.h"

class DataPublish: public DataUtility{
public:
    std::vector<ros::Publisher> pubCameraColors;
    std::vector<ros::Publisher> pubCameraDepths;
    std::vector<ros::Publisher> pubCameraPointClouds;
    std::vector<ros::Publisher> pubArmJointStates;
    std::vector<ros::Publisher> pubArmEndPoses;
    std::vector<ros::Publisher> pubRobotBaseVels;

    std::vector<ros::Publisher> pubCameraColorConfigs;
    std::vector<ros::Publisher> pubCameraDepthConfigs;
    std::vector<ros::Publisher> pubCameraPointCloudConfigs;
    std::vector<ros::Publisher> pubArmJointStateConfigs;
    std::vector<ros::Publisher> pubArmEndPoseConfigs;
    std::vector<ros::Publisher> pubRobotBaseVelConfigs;

    std::vector<sem_t> cameraColorSems;
    std::vector<sem_t> cameraDepthSems;
    std::vector<sem_t> cameraPointCloudSems;
    std::vector<sem_t> armJointStateSems;
    std::vector<sem_t> armEndPoseSems;
    std::vector<sem_t> robotBaseVelSems;

    std::vector<std::ifstream> syncFileCameraColors;
    std::vector<std::ifstream> syncFileCameraDepths;
    std::vector<std::ifstream> syncFileCameraPointClouds;
    std::vector<std::ifstream> syncFileArmJointStates;
    std::vector<std::ifstream> syncFileArmEndPoses;
    std::vector<std::ifstream> syncFileRobotBaseVels;

    std::vector<std::thread*> cameraColorPublishingThreads;
    std::vector<std::thread*> cameraDepthPublishingThreads;
    std::vector<std::thread*> cameraPointCloudPublishingThreads;
    std::vector<std::thread*> armJointStatePublishingThreads;
    std::vector<std::thread*> armEndPosePublishingThreads;
    std::vector<std::thread*> robotBaseVelPublishingThreads;

    std::thread* activatingThread;

    float publishRate;
    bool publishFirst;

    DataPublish(std::string datasetDir, int episodeIndex, bool publishFirstParam, float publishRateParam): DataUtility(datasetDir, episodeIndex) {
        publishRate = publishRateParam;
        publishFirst = publishFirstParam;

        syncFileCameraColors = std::vector<std::ifstream>(cameraColorNames.size());
        syncFileCameraDepths = std::vector<std::ifstream>(cameraDepthNames.size());
        syncFileCameraPointClouds = std::vector<std::ifstream>(cameraPointCloudNames.size());
        syncFileArmJointStates = std::vector<std::ifstream>(armJointStateNames.size());
        syncFileArmEndPoses = std::vector<std::ifstream>(armEndPoseNames.size());
        syncFileRobotBaseVels = std::vector<std::ifstream>(robotBaseVelNames.size());

        cameraColorSems = std::vector<sem_t>(cameraColorNames.size());
        cameraDepthSems = std::vector<sem_t>(cameraDepthNames.size());
        cameraPointCloudSems = std::vector<sem_t>(cameraPointCloudNames.size());
        armJointStateSems = std::vector<sem_t>(armJointStateNames.size());
        armEndPoseSems = std::vector<sem_t>(armEndPoseNames.size());
        robotBaseVelSems = std::vector<sem_t>(robotBaseVelNames.size());

        for(int i = 0; i < cameraColorNames.size(); i++){
            pubCameraColors.push_back(nh.advertise<sensor_msgs::Image>(cameraColorPublishTopics[i], 2000));
            pubCameraColorConfigs.push_back(nh.advertise<sensor_msgs::CameraInfo>(cameraColorConfigPublishTopics[i], 2000));
            if(cameraColorToPublishs.at(i)){
                syncFileCameraColors.at(i).open(cameraColorDirs.at(i)+"/sync.txt");
                sem_init(&cameraColorSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            pubCameraDepths.push_back(nh.advertise<sensor_msgs::Image>(cameraDepthPublishTopics[i], 2000));
            pubCameraDepthConfigs.push_back(nh.advertise<sensor_msgs::CameraInfo>(cameraDepthConfigPublishTopics[i], 2000));
            if(cameraDepthToPublishs.at(i)){
                syncFileCameraDepths.at(i).open(cameraDepthDirs.at(i)+"/sync.txt");
                sem_init(&cameraDepthSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            pubCameraPointClouds.push_back(nh.advertise<sensor_msgs::PointCloud2>(cameraPointCloudPublishTopics[i], 2000));
            pubCameraPointCloudConfigs.push_back(nh.advertise<sensor_msgs::CameraInfo>(cameraPointCloudConfigPublishTopics[i], 2000));
            if(cameraPointCloudToPublishs.at(i)){
                syncFileCameraPointClouds.at(i).open(cameraPointCloudDirs.at(i)+"/sync.txt");
                sem_init(&cameraPointCloudSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            pubArmJointStates.push_back(nh.advertise<sensor_msgs::JointState>(armJointStatePublishTopics[i], 2000));
            if(armJointStateToPublishs.at(i)){
                syncFileArmJointStates.at(i).open(armJointStateDirs.at(i)+"/sync.txt");
                sem_init(&armJointStateSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            pubArmEndPoses.push_back(nh.advertise<geometry_msgs::PoseStamped>(armEndPosePublishTopics[i], 2000));
            if(armEndPoseToPublishs.at(i)){
                syncFileArmEndPoses.at(i).open(armEndPoseDirs.at(i)+"/sync.txt");
                sem_init(&armEndPoseSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            pubRobotBaseVels.push_back(nh.advertise<nav_msgs::Odometry>(robotBaseVelPublishTopics[i], 2000));
            if(robotBaseVelToPublishs.at(i)){
                syncFileRobotBaseVels.at(i).open(robotBaseVelDirs.at(i)+"/sync.txt");
                sem_init(&robotBaseVelSems.at(i), 0, 0);
            }
        }
    }

    void cameraColorPublishing(const int index){
        std::string time0 = "";
        while(ros::ok()){
            std::string time;
            if(publishFirst){
                if(time0 == "")
                    getline(syncFileCameraColors.at(index), time0);
                time = time0;
            }else{
                if(!getline(syncFileCameraColors.at(index), time))
                    break;
            }
            sem_wait(&cameraColorSems.at(index));
            cv::Mat image = cv::imread(cameraColorDirs.at(index) + "/" + time,cv::IMREAD_COLOR);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
            pubCameraColors.at(index).publish(*msg);
        }
    }

    void cameraDepthPublishing(const int index){
        std::string time0 = "";
        while(ros::ok()){
            std::string time;
            if(publishFirst){
                if(time0 == "")
                    getline(syncFileCameraDepths.at(index), time0);
                time = time0;
            }else{
                if(!getline(syncFileCameraDepths.at(index), time))
                    break;
            }
            sem_wait(&cameraDepthSems.at(index));
            cv::Mat image = cv::imread(cameraDepthDirs.at(index) + "/" + time,cv::IMREAD_ANYDEPTH);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
            pubCameraDepths.at(index).publish(msg);
        }
    }

    void cameraPointCloudPublishing(const int index){
        std::string time0 = "";
        while(ros::ok()){
            std::string time;
            if(publishFirst){
                if(time0 == "")
                    getline(syncFileCameraPointClouds.at(index), time0);
                time = time0;
            }else{
                if(!getline(syncFileCameraPointClouds.at(index), time))
                    break;
            }
            sem_wait(&cameraPointCloudSems.at(index));
            pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(cameraPointCloudDirs.at(index) + "/" + time, pointcloud);
            sensor_msgs::PointCloud2 cloudMsg;
            pcl::toROSMsg(pointcloud, cloudMsg);
            pubCameraPointClouds.at(index).publish(cloudMsg);
        }
    }

    void armJointStatePublishing(const int index){
        std::string time0 = "";
        while(ros::ok()){
            std::string time;
            if(publishFirst){
                if(time0 == "")
                    getline(syncFileArmJointStates.at(index), time0);
                time = time0;
            }else{
                if(!getline(syncFileArmJointStates.at(index), time))
                    break;
            }
            sem_wait(&armJointStateSems.at(index));
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(armJointStateDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            Json::Value effort = root["effort"];
            std::vector<double> effortData;
            for (int i = 0; i < effort.size(); i++)
                effortData.push_back(effort[i].asDouble());
            Json::Value position = root["position"];
            std::vector<double> positionData;
            for (int i = 0; i < position.size(); i++)
                positionData.push_back(position[i].asDouble());
            Json::Value velocity = root["velocity"];
            std::vector<double> velocityData;
            for (int i = 0; i < velocity.size(); i++)
                velocityData.push_back(velocity[i].asDouble());
            sensor_msgs::JointState msg;
            msg.position = positionData;
            pubArmJointStates.at(index).publish(msg);
        }
    }

    void armEndPosePublishing(const int index){
        std::string time0 = "";
        while(ros::ok()){
            std::string time;
            if(publishFirst){
                if(time0 == "")
                    getline(syncFileArmEndPoses.at(index), time0);
                time = time0;
            }else{
                if(!getline(syncFileArmEndPoses.at(index), time))
                    break;
            }
            sem_wait(&armEndPoseSems.at(index));
            geometry_msgs::PoseStamped msg;
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(armEndPoseDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.pose.position.x = root["x"].asDouble();
            msg.pose.position.y = root["y"].asDouble();
            msg.pose.position.z = root["z"].asDouble();
            msg.pose.orientation.x = root["roll"].asDouble();
            msg.pose.orientation.y = root["pitch"].asDouble();
            msg.pose.orientation.z = root["yaw"].asDouble();
            msg.pose.orientation.w = root["grasper"].asDouble();
            pubArmEndPoses.at(index).publish(msg);
        }
    }

    void robotBaseVelPublishing(const int index){
        std::string time0 = "";
        while(ros::ok()){
            std::string time;
            if(publishFirst){
                if(time0 == "")
                    getline(syncFileRobotBaseVels.at(index), time0);
                time = time0;
            }else{
                if(!getline(syncFileRobotBaseVels.at(index), time))
                    break;
            }
            sem_wait(&robotBaseVelSems.at(index));
            nav_msgs::Odometry msg;
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(robotBaseVelDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.twist.twist.linear.x = root["linear"]["x"].asDouble();
            msg.twist.twist.linear.y = root["linear"]["y"].asDouble();
            msg.twist.twist.angular.z = root["angular"]["z"].asDouble();
            pubRobotBaseVels.at(index).publish(msg);
        }
    }

    void activating(){
        ros::Rate rate(publishRate);
        while(ros::ok()){
            for(int i = 0; i < cameraColorNames.size(); i++){
                if(cameraColorToPublishs.at(i))
                    sem_post(&cameraColorSems.at(i));
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                if(cameraDepthToPublishs.at(i))
                    sem_post(&cameraDepthSems.at(i));
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                if(cameraPointCloudToPublishs.at(i))
                    sem_post(&cameraPointCloudSems.at(i));
            }
            for(int i = 0; i < armJointStateNames.size(); i++){
                if(armJointStateToPublishs.at(i))
                    sem_post(&armJointStateSems.at(i));
            }
            for(int i = 0; i < armEndPoseNames.size(); i++){
                if(armEndPoseToPublishs.at(i))
                    sem_post(&armEndPoseSems.at(i));
            }
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                if(robotBaseVelToPublishs.at(i))
                    sem_post(&robotBaseVelSems.at(i));
            }
            rate.sleep();
        }
    }

    void join(){
        for(int i = 0; i < cameraColorPublishingThreads.size(); i++){
            cameraColorPublishingThreads.at(i)->join();
            delete cameraColorPublishingThreads.at(i);
            cameraColorPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraDepthPublishingThreads.size(); i++){
            cameraDepthPublishingThreads.at(i)->join();
            delete cameraDepthPublishingThreads.at(i);
            cameraDepthPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraPointCloudPublishingThreads.size(); i++){
            cameraPointCloudPublishingThreads.at(i)->join();
            delete cameraPointCloudPublishingThreads.at(i);
            cameraPointCloudPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armJointStatePublishingThreads.size(); i++){
            armJointStatePublishingThreads.at(i)->join();
            delete armJointStatePublishingThreads.at(i);
            armJointStatePublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armEndPosePublishingThreads.size(); i++){
            armEndPosePublishingThreads.at(i)->join();
            delete armEndPosePublishingThreads.at(i);
            armEndPosePublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < robotBaseVelPublishingThreads.size(); i++){
            robotBaseVelPublishingThreads.at(i)->join();
            delete robotBaseVelPublishingThreads.at(i);
            robotBaseVelPublishingThreads.at(i) = nullptr;
        }
        ros::shutdown();
    }

    void run(){
        for(int i = 0; i < cameraColorNames.size(); i++){
            if(cameraColorToPublishs.at(i))
                cameraColorPublishingThreads.push_back(new std::thread(&DataPublish::cameraColorPublishing, this, i));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            if(cameraDepthToPublishs.at(i))
                cameraDepthPublishingThreads.push_back(new std::thread(&DataPublish::cameraDepthPublishing, this, i));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            if(cameraPointCloudToPublishs.at(i))
                cameraPointCloudPublishingThreads.push_back(new std::thread(&DataPublish::cameraPointCloudPublishing, this, i));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            if(armJointStateToPublishs.at(i))
                armJointStatePublishingThreads.push_back(new std::thread(&DataPublish::armJointStatePublishing, this, i));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            if(armEndPoseToPublishs.at(i))
                armEndPosePublishingThreads.push_back(new std::thread(&DataPublish::armEndPosePublishing, this, i));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            if(robotBaseVelToPublishs.at(i))
                robotBaseVelPublishingThreads.push_back(new std::thread(&DataPublish::robotBaseVelPublishing, this, i));
        }
        activatingThread = new std::thread(&DataPublish::activating, this);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_sync");
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    float publishRate;
    bool publishFirst;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    nh.param<float>("publishRate", publishRate, 30);
    nh.param<bool>("publishFirst", publishFirst, false);
    ROS_INFO("\033[1;32m----> data publish Started.\033[0m");
    DataPublish dataPublish(datasetDir, episodeIndex, publishFirst, publishRate);
    dataPublish.run();
    dataPublish.join();
    std::cout<<"Done"<<std::endl;
    return 0;
}
