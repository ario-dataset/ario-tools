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
#include "blockingDeque.h"
#include <mutex>
#include <math.h>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

class DataCapture: public DataUtility{
public:
    std::vector<ros::Subscriber> subCameraColors;
    std::vector<ros::Subscriber> subCameraDepths;
    std::vector<ros::Subscriber> subCameraPointClouds;
    std::vector<ros::Subscriber> subArmJointStates;
    std::vector<ros::Subscriber> subArmEndPoses;
    std::vector<ros::Subscriber> subRobotBaseVels;

    std::vector<ros::Subscriber> subCameraColorConfigs;
    std::vector<ros::Subscriber> subCameraDepthConfigs;
    std::vector<ros::Subscriber> subCameraPointCloudConfigs;
    std::vector<ros::Subscriber> subArmJointStateConfigs;
    std::vector<ros::Subscriber> subArmEndPoseConfigs;
    std::vector<ros::Subscriber> subRobotBaseVelConfigs;

    std::vector<BlockingDeque<sensor_msgs::Image>> cameraColorMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::Image>> cameraDepthMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::PointCloud2>> cameraPointCloudMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::JointState>> armJointStateMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::PoseStamped>> armEndPoseMsgDeques;
    std::vector<BlockingDeque<nav_msgs::Odometry>> robotBaseVelMsgDeques;

    std::vector<int> cameraColorMsgCounts;
    std::vector<int> cameraDepthMsgCounts;
    std::vector<int> cameraPointCloudMsgCounts;
    std::vector<int> armJointStateMsgCounts;
    std::vector<int> armEndPoseMsgCounts;
    std::vector<int> robotBaseVelMsgCounts;

    BlockingDeque<std::vector<sensor_msgs::Image>> cameraColorMsgSavingVecs;
    BlockingDeque<std::vector<sensor_msgs::Image>> cameraDepthMsgSavingVecs;
    BlockingDeque<std::vector<sensor_msgs::PointCloud2>> cameraPointCloudMsgSavingVecs;
    BlockingDeque<std::vector<sensor_msgs::JointState>> armJointStateMsgSavingVecs;
    BlockingDeque<std::vector<geometry_msgs::PoseStamped>> armEndPoseMsgSavingVecs;
    BlockingDeque<std::vector<nav_msgs::Odometry>> robotBaseVelMsgSavingVecs;

    std::thread* savingThread;
    std::thread* keyboardInterruptCheckingThread;
    std::thread* capturingThread;
    bool keyboardInterrupt = false;
    bool captureStop = false;
    std::mutex captureStopMtx;

    DataCapture(std::string datasetDir, int episodeIndex): DataUtility(datasetDir, episodeIndex) {
        int unused = system((std::string("rm -r ") + episodeDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraColorDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraDepthDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraPointCloudDir).c_str());
        unused = system((std::string("mkdir -p ") + armJointStateDir).c_str());
        unused = system((std::string("mkdir -p ") + armEndPoseDir).c_str());
        unused = system((std::string("mkdir -p ") + robotBaseVelDir).c_str());

        cameraColorMsgDeques = std::vector<BlockingDeque<sensor_msgs::Image>>(cameraColorNames.size());
        cameraDepthMsgDeques = std::vector<BlockingDeque<sensor_msgs::Image>>(cameraDepthNames.size());
        cameraPointCloudMsgDeques = std::vector<BlockingDeque<sensor_msgs::PointCloud2>>(cameraPointCloudNames.size());
        armJointStateMsgDeques = std::vector<BlockingDeque<sensor_msgs::JointState>>(armJointStateNames.size());
        armEndPoseMsgDeques = std::vector<BlockingDeque<geometry_msgs::PoseStamped>>(armEndPoseNames.size());
        robotBaseVelMsgDeques = std::vector<BlockingDeque<nav_msgs::Odometry>>(robotBaseVelNames.size());
    
        cameraColorMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        robotBaseVelMsgCounts = std::vector<int>(robotBaseVelNames.size(), 0);
        for(int i = 0; i < cameraColorNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraColorDirs.at(i)).c_str());
            subCameraColors.push_back(nh.subscribe<sensor_msgs::Image>(cameraColorTopics[i], 2000, boost::bind(&DataCapture::cameraColorHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            subCameraColorConfigs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cameraColorConfigTopics[i], 2000, boost::bind(&DataCapture::cameraColorConfigHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraDepthDirs.at(i)).c_str());
            subCameraDepths.push_back(nh.subscribe<sensor_msgs::Image>(cameraDepthTopics[i], 2000, boost::bind(&DataCapture::cameraDepthHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            subCameraDepthConfigs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cameraDepthConfigTopics[i], 2000, boost::bind(&DataCapture::cameraDepthConfigHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraPointCloudDirs.at(i)).c_str());
            subCameraPointClouds.push_back(nh.subscribe<sensor_msgs::PointCloud2>(cameraPointCloudTopics[i], 2000, boost::bind(&DataCapture::cameraPointCloudHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            subCameraPointCloudConfigs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cameraPointCloudConfigTopics[i], 2000, boost::bind(&DataCapture::cameraPointCloudConfigHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            unused = system((std::string("mkdir -p ") + armJointStateDirs.at(i)).c_str());
            subArmJointStates.push_back(nh.subscribe<sensor_msgs::JointState>(armJointStateTopics[i], 2000, boost::bind(&DataCapture::armJointStateHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            unused = system((std::string("mkdir -p ") + armEndPoseDirs.at(i)).c_str());
            subArmEndPoses.push_back(nh.subscribe<geometry_msgs::PoseStamped>(armEndPoseTopics[i], 2000, boost::bind(&DataCapture::armEndPoseHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            unused = system((std::string("mkdir -p ") + robotBaseVelDirs.at(i)).c_str());
            subRobotBaseVels.push_back(nh.subscribe<nav_msgs::Odometry>(robotBaseVelTopics[i], 2000, boost::bind(&DataCapture::robotBaseVelHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
    }

    void run(){
        capturingThread = new std::thread(&DataCapture::capturing, this);
        savingThread = new std::thread(&DataCapture::saving, this);
        keyboardInterruptCheckingThread = new std::thread(&DataCapture::keyboardInterruptChecking, this);
    }

    void join(){
        capturingThread->join();
        savingThread->join();
        keyboardInterruptCheckingThread->join();
        delete capturingThread;
        delete savingThread;
        delete keyboardInterruptCheckingThread;
    }

    void cameraColorHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cameraColorMsgDeques.at(index).push_back(*msg);
        cameraColorMsgCounts.at(index) += 1;
        // std::cout<<"cameraColorMsgCounts:"<<index<<" "<<cameraColorMsgCounts.at(index)<<std::endl;
    }

    void cameraDepthHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cameraDepthMsgDeques.at(index).push_back(*msg);
        cameraDepthMsgCounts.at(index) += 1;
        // std::cout<<"cameraDepthMsgCounts:"<<index<<" "<<cameraDepthMsgCounts.at(index)<<std::endl;
    }

    void cameraPointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& msg, const int& index){
        cameraPointCloudMsgDeques.at(index).push_back(*msg);
        cameraPointCloudMsgCounts.at(index) += 1;
        // std::cout<<"cameraPointCloudMsgCounts:"<<index<<" "<<cameraPointCloudMsgCounts.at(index)<<std::endl;
    }

    void armJointStateHandler(const sensor_msgs::JointState::ConstPtr& msg, const int& index){
        armJointStateMsgDeques.at(index).push_back(*msg);
        armJointStateMsgCounts.at(index) += 1;
        // std::cout<<"armJointStateMsgCounts:"<<index<<" "<<armJointStateMsgCounts.at(index)<<std::endl;
    }

    void armEndPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg, const int& index){
        armEndPoseMsgDeques.at(index).push_back(*msg);
        armEndPoseMsgCounts.at(index) += 1;
        // std::cout<<"armEndPoseMsgCounts:"<<index<<" "<<armEndPoseMsgCounts.at(index)<<std::endl;
    }

    void robotBaseVelHandler(const nav_msgs::Odometry::ConstPtr& msg, const int& index){
        robotBaseVelMsgDeques.at(index).push_back(*msg);
        robotBaseVelMsgCounts.at(index) += 1;
        // std::cout<<"robotBaseVelMsgCounts:"<<index<<" "<<robotBaseVelMsgCounts.at(index)<<std::endl;
    }

    void cameraColorConfigHandler(const sensor_msgs::CameraInfo::ConstPtr& msg, const int& index){
        tf::TransformListener listener;
		tf::StampedTransform transform;
		while(ros::ok() && msg->header.frame_id != ""){
            try{
                listener.waitForTransform(cameraColorParentFrames.at(index), msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraColorParentFrames.at(index), msg->header.frame_id, ros::Time(0), transform);
                break;
            }catch(tf::TransformException &ex){
                ros::Duration(1.0).sleep();
                continue;
            }
		}
		double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        double roll, pitch, yaw;
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        std::ofstream file(cameraColorDirs.at(index) + "/config.txt");
        file<<"height: "<<std::to_string(msg->height)<<std::endl;
        file<<"width: "<<std::to_string(msg->width)<<std::endl;
        file<<"distortion_model: "<<msg->distortion_model<<std::endl;
        file<<"D: [";
        for(int j = 0; j < msg->D.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->D.at(j);
        }
        file<<"]"<<std::endl;
        file<<"K: [";
        for(int j = 0; j < msg->K.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->K.at(j);
        }
        file<<"]"<<std::endl;
        file<<"R: [";
        for(int j = 0; j < msg->R.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->R.at(j);
        }
        file<<"]"<<std::endl;
        file<<"P: [";
        for(int j = 0; j < msg->P.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->P.at(j);
        }
        file<<"]"<<std::endl;
        file<<"binning_x: "<<std::to_string(msg->binning_x)<<std::endl;
        file<<"binning_y: "<<std::to_string(msg->binning_y)<<std::endl;
        file<<"roi_x_offset: "<<std::to_string(msg->roi.x_offset)<<std::endl;
        file<<"roi_y_offset: "<<std::to_string(msg->roi.y_offset)<<std::endl;
        file<<"roi_height: "<<std::to_string(msg->roi.height)<<std::endl;
        file<<"roi_width: "<<std::to_string(msg->roi.width)<<std::endl;
        file<<"roi_do_rectify: "<<std::to_string(msg->roi.do_rectify)<<std::endl;
        file<<"x_in_parent_frame: "<<std::to_string(x)<<std::endl;
        file<<"y_in_parent_frame: "<<std::to_string(y)<<std::endl;
        file<<"z_in_parent_frame: "<<std::to_string(z)<<std::endl;
        file<<"roll_in_parent_frame: "<<std::to_string(roll)<<std::endl;
        file<<"pitch_in_parent_frame: "<<std::to_string(pitch)<<std::endl;
        file<<"yaw_in_parent_frame: "<<std::to_string(yaw)<<std::endl;
        file.close();
    }

    void cameraDepthConfigHandler(const sensor_msgs::CameraInfo::ConstPtr& msg, const int& index){
        tf::TransformListener listener;
		tf::StampedTransform transform;
		while(ros::ok() && msg->header.frame_id != ""){
            try
            {
                listener.waitForTransform(cameraDepthParentFrames.at(index), msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraDepthParentFrames.at(index), msg->header.frame_id, ros::Time(0), transform);
                break;
            }
            catch (tf::TransformException &ex)
            {
                ros::Duration(1.0).sleep();
                continue;
            }
		}
		double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        double roll, pitch, yaw;
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        std::ofstream file(cameraDepthDirs.at(index) + "/config.txt");
        file<<"height: "<<std::to_string(msg->height)<<std::endl;
        file<<"width: "<<std::to_string(msg->width)<<std::endl;
        file<<"distortion_model: "<<msg->distortion_model<<std::endl;
        file<<"D: [";
        for(int j = 0; j < msg->D.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->D.at(j);
        }
        file<<"]"<<std::endl;
        file<<"K: [";
        for(int j = 0; j < msg->K.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->K.at(j);
        }
        file<<"]"<<std::endl;
        file<<"R: [";
        for(int j = 0; j < msg->R.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->R.at(j);
        }
        file<<"]"<<std::endl;
        file<<"P: [";
        for(int j = 0; j < msg->P.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->P.at(j);
        }
        file<<"]"<<std::endl;
        file<<"binning_x: "<<std::to_string(msg->binning_x)<<std::endl;
        file<<"binning_y: "<<std::to_string(msg->binning_y)<<std::endl;
        file<<"roi_x_offset: "<<std::to_string(msg->roi.x_offset)<<std::endl;
        file<<"roi_y_offset: "<<std::to_string(msg->roi.y_offset)<<std::endl;
        file<<"roi_height: "<<std::to_string(msg->roi.height)<<std::endl;
        file<<"roi_width: "<<std::to_string(msg->roi.width)<<std::endl;
        file<<"roi_do_rectify: "<<std::to_string(msg->roi.do_rectify)<<std::endl;
        file<<"x_in_parent_frame: "<<std::to_string(x)<<std::endl;
        file<<"y_in_parent_frame: "<<std::to_string(y)<<std::endl;
        file<<"z_in_parent_frame: "<<std::to_string(z)<<std::endl;
        file<<"roll_in_parent_frame: "<<std::to_string(roll)<<std::endl;
        file<<"pitch_in_parent_frame: "<<std::to_string(pitch)<<std::endl;
        file<<"yaw_in_parent_frame: "<<std::to_string(yaw)<<std::endl;
        file.close();
    }

    void cameraPointCloudConfigHandler(const sensor_msgs::CameraInfo::ConstPtr& msg, const int& index){
        tf::TransformListener listener;
		tf::StampedTransform transform;
		while(ros::ok() && msg->header.frame_id != ""){
            try
            {
                listener.waitForTransform(cameraPointCloudParentFrames.at(index), msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraPointCloudParentFrames.at(index), msg->header.frame_id, ros::Time(0), transform);
                break;
            }
            catch (tf::TransformException &ex)
            {
                ros::Duration(1.0).sleep();
                continue;
            }
		}
		double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        double roll, pitch, yaw;        
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        std::ofstream file(cameraPointCloudDirs.at(index) + "/config.txt");
        file<<"height: "<<std::to_string(msg->height)<<std::endl;
        file<<"width: "<<std::to_string(msg->width)<<std::endl;
        file<<"distortion_model: "<<msg->distortion_model<<std::endl;
        file<<"D: [";
        for(int j = 0; j < msg->D.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->D.at(j);
        }
        file<<"]"<<std::endl;
        file<<"K: [";
        for(int j = 0; j < msg->K.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->K.at(j);
        }
        file<<"]"<<std::endl;
        file<<"R: [";
        for(int j = 0; j < msg->R.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->R.at(j);
        }
        file<<"]"<<std::endl;
        file<<"P: [";
        for(int j = 0; j < msg->P.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->P.at(j);
        }
        file<<"]"<<std::endl;
        file<<"binning_x: "<<std::to_string(msg->binning_x)<<std::endl;
        file<<"binning_y: "<<std::to_string(msg->binning_y)<<std::endl;
        file<<"roi_x_offset: "<<std::to_string(msg->roi.x_offset)<<std::endl;
        file<<"roi_y_offset: "<<std::to_string(msg->roi.y_offset)<<std::endl;
        file<<"roi_height: "<<std::to_string(msg->roi.height)<<std::endl;
        file<<"roi_width: "<<std::to_string(msg->roi.width)<<std::endl;
        file<<"roi_do_rectify: "<<std::to_string(msg->roi.do_rectify)<<std::endl;
        file<<"x_in_parent_frame: "<<std::to_string(x)<<std::endl;
        file<<"y_in_parent_frame: "<<std::to_string(y)<<std::endl;
        file<<"z_in_parent_frame: "<<std::to_string(z)<<std::endl;
        file<<"roll_in_parent_frame: "<<std::to_string(roll)<<std::endl;
        file<<"pitch_in_parent_frame: "<<std::to_string(pitch)<<std::endl;
        file<<"yaw_in_parent_frame: "<<std::to_string(yaw)<<std::endl;
        file.close();
    }

    void capturing(){
        int frameCount = 0;
        ros::Rate rate(30);
        while(ros::ok()){
            captureStopMtx.lock();
            if(frameCount >= captureFrameNum || keyboardInterrupt){
                captureStop = true;
                captureStopMtx.unlock();
                break;
            }
            captureStopMtx.unlock();
            double frameTime = INFINITY;
            for(int i = 0; i < cameraColorNames.size(); i++){
                if(!cameraColorToSyncs.at(i))
                    continue;
                double time = cameraColorMsgDeques.at(i).back().header.stamp.toSec();
                if(time < frameTime)
                    frameTime = time;
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                if(!cameraDepthToSyncs.at(i))
                    continue;
                double time = cameraDepthMsgDeques.at(i).back().header.stamp.toSec();
                if(time < frameTime)
                    frameTime = time;
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                if(!cameraPointCloudToSyncs.at(i))
                    continue;
                double time = cameraPointCloudMsgDeques.at(i).back().header.stamp.toSec();
                if(time < frameTime)
                    frameTime = time;
            }
            for(int i = 0; i < armJointStateNames.size(); i++){
                if(!armJointStateToSyncs.at(i))
                    continue;
                double time = armJointStateMsgDeques.at(i).back().header.stamp.toSec();
                if(time < frameTime)
                    frameTime = time;
            }
            for(int i = 0; i < armEndPoseNames.size(); i++){
                if(!armEndPoseToSyncs.at(i))
                    continue;
                double time = armEndPoseMsgDeques.at(i).back().header.stamp.toSec();
                if(time < frameTime)
                    frameTime = time;
            }
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                if(!robotBaseVelToSyncs.at(i))
                    continue;
                double time = robotBaseVelMsgDeques.at(i).back().header.stamp.toSec();
                if(time < frameTime)
                    frameTime = time;
            }
            std::vector<sensor_msgs::Image> cameraColors;
            for(int i = 0; i < cameraColorNames.size(); i++){
                if(!cameraColorToSyncs.at(i))
                    continue;
                cameraColors.push_back(cameraColorMsgDeques.at(i).getRecentItem(frameTime));
            }
            cameraColorMsgSavingVecs.push_back(cameraColors);
            std::vector<sensor_msgs::Image> cameraDepths;
            for(int i = 0; i < cameraDepthNames.size(); i++){
                if(!cameraDepthToSyncs.at(i))
                    continue;
                cameraDepths.push_back(cameraDepthMsgDeques.at(i).getRecentItem(frameTime));
            }
            cameraDepthMsgSavingVecs.push_back(cameraDepths);
            std::vector<sensor_msgs::PointCloud2> cameraPointClouds;
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                if(!cameraPointCloudToSyncs.at(i))
                    continue;
                cameraPointClouds.push_back(cameraPointCloudMsgDeques.at(i).getRecentItem(frameTime));
            }
            cameraPointCloudMsgSavingVecs.push_back(cameraPointClouds);
            std::vector<sensor_msgs::JointState> armJointStates;
            for(int i = 0; i < armJointStateNames.size(); i++){
                if(!armJointStateToSyncs.at(i))
                    continue;
                armJointStates.push_back(armJointStateMsgDeques.at(i).getRecentItem(frameTime));
            }
            armJointStateMsgSavingVecs.push_back(armJointStates);
            std::vector<geometry_msgs::PoseStamped> armEndPoses;
            for(int i = 0; i < armEndPoseNames.size(); i++){
                if(!armEndPoseToSyncs.at(i))
                    continue;
                armEndPoses.push_back(armEndPoseMsgDeques.at(i).getRecentItem(frameTime));
            }
            armEndPoseMsgSavingVecs.push_back(armEndPoses);
            std::vector<nav_msgs::Odometry> robotBaseVels;
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                if(!robotBaseVelToSyncs.at(i))
                    continue;
                robotBaseVels.push_back(robotBaseVelMsgDeques.at(i).getRecentItem(frameTime));
            }            
            robotBaseVelMsgSavingVecs.push_back(robotBaseVels);
            frameCount += 1;
            // rate.sleep();
        }
    }

    void saving(){
        int frameCount = 0;
        while(ros::ok()){
            captureStopMtx.lock();
            if(captureStop && cameraColorMsgSavingVecs.size() == 0){
                captureStopMtx.unlock();
                ros::shutdown();
                break;
            }
            captureStopMtx.unlock();
            std::vector<sensor_msgs::Image> cameraColors = cameraColorMsgSavingVecs.pop_front();
            for(int i = 0; i < cameraColorNames.size(); i++){
                if(!cameraColorToSyncs.at(i))
                    continue;
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(cameraColors.at(i), sensor_msgs::image_encodings::BGR8);
                cv::imwrite(cameraColorDirs.at(i) + "/" + std::to_string(frameCount) + ".png", cv_ptr->image);
            }
            std::vector<sensor_msgs::Image> cameraDepths = cameraDepthMsgSavingVecs.pop_front();
            for(int i = 0; i < cameraDepthNames.size(); i++){
                if(!cameraDepthToSyncs.at(i))
                    continue;
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(cameraDepths.at(i), sensor_msgs::image_encodings::TYPE_16UC1);
                cv::imwrite(cameraDepthDirs.at(i) + "/" + std::to_string(frameCount) + ".png", cv_ptr->image);
            }
            std::vector<sensor_msgs::PointCloud2> cameraPointClouds = cameraPointCloudMsgSavingVecs.pop_front();
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                if(!cameraPointCloudToSyncs.at(i))
                    continue;
                pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
                pcl::fromROSMsg(cameraPointClouds.at(i), pointcloud);
                pcl::io::savePCDFileBinary(cameraPointCloudDirs.at(i) + "/" + std::to_string(frameCount) + ".pcd", pointcloud);
            }
            std::vector<sensor_msgs::JointState> armJointStates = armJointStateMsgSavingVecs.pop_front();
            for(int i = 0; i < armJointStateNames.size(); i++){
                if(!armJointStateToSyncs.at(i))
                    continue;
                std::ofstream file(armJointStateDirs.at(i) + "/" + std::to_string(frameCount) + ".txt");
                file<<"effort: [";
                for(int j = 0; j < armJointStates.at(i).effort.size(); j++){
                    if(j != 0)
                        file<<", ";
                    file<<armJointStates.at(i).effort.at(j);
                }
                file<<"]"<<std::endl;
                file<<"position: [";
                for(int j = 0; j < armJointStates.at(i).position.size(); j++){
                    if(j != 0)
                        file<<", ";
                    file<<" "<<armJointStates.at(i).position.at(j);
                }
                file<<"]"<<std::endl;
                file<<std::endl;
                file<<"velocity: [";
                for(int j = 0; j < armJointStates.at(i).velocity.size(); j++){
                    if(j != 0)
                        file<<", ";
                    file<<" "<<armJointStates.at(i).velocity.at(j);
                }
                file<<"]"<<std::endl;
                file.close();
            }
            std::vector<geometry_msgs::PoseStamped> armEndPoses = armEndPoseMsgSavingVecs.pop_front();
            for(int i = 0; i < armEndPoseNames.size(); i++){
                if(!armEndPoseToSyncs.at(i))
                    continue;
                std::ofstream file(armEndPoseDirs.at(i) + "/" + std::to_string(frameCount) + ".txt");
                file<<"x: "<<armEndPoses.at(i).pose.position.x<<std::endl;
                file<<"y: "<<armEndPoses.at(i).pose.position.y<<std::endl;
                file<<"z: "<<armEndPoses.at(i).pose.position.z<<std::endl;
                file<<"roll: "<<armEndPoses.at(i).pose.orientation.x<<std::endl;
                file<<"pitch: "<<armEndPoses.at(i).pose.orientation.y<<std::endl;
                file<<"yaw: "<<armEndPoses.at(i).pose.orientation.z<<std::endl;
                file<<"grasper: "<<armEndPoses.at(i).pose.orientation.w<<std::endl;
                file.close();
            }
            std::vector<nav_msgs::Odometry> robotBaseVels = robotBaseVelMsgSavingVecs.pop_front();
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                if(!robotBaseVelToSyncs.at(i))
                    continue;
                std::ofstream file(robotBaseVelDirs.at(i) + "/" + std::to_string(frameCount) + ".txt");
                file<<"linear_x: "<<robotBaseVels.at(i).twist.twist.linear.x<<std::endl;
                file<<"linear_y: "<<robotBaseVels.at(i).twist.twist.linear.y<<std::endl;
                file<<"angular_z: "<<robotBaseVels.at(i).twist.twist.angular.x<<std::endl;
                file.close();
            }
            frameCount += 1;
            std::cout<<"frame num:"<<frameCount<<std::endl;
        }
    }

    void keyboardInterruptChecking(){
        std::string line;
        if (std::getline(std::cin, line)) {
            std::cout<<"Keyboard Interrupt"<<std::endl;
            std::lock_guard<std::mutex> lock(captureStopMtx);
            int count = 0;
            keyboardInterrupt = true;
            for(int i = 0; i < cameraColorNames.size(); i++){
                count += cameraColorMsgCounts.at(i);
                std::cout<<cameraColorTopics.at(i)<<": "<<cameraColorMsgCounts.at(i)<<std::endl;
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                count += cameraDepthMsgCounts.at(i);
                std::cout<<cameraDepthTopics.at(i)<<": "<<cameraDepthMsgCounts.at(i)<<std::endl;
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                count += cameraPointCloudMsgCounts.at(i);
                std::cout<<cameraPointCloudTopics.at(i)<<": "<<cameraPointCloudMsgCounts.at(i)<<std::endl;
            }
            for(int i = 0; i < armJointStateNames.size(); i++){
                count += armJointStateMsgCounts.at(i);
                std::cout<<armJointStateTopics.at(i)<<": "<<armJointStateMsgCounts.at(i)<<std::endl;
            }
            for(int i = 0; i < armEndPoseNames.size(); i++){
                count += armEndPoseMsgCounts.at(i);
                std::cout<<armEndPoseTopics.at(i)<<": "<<armEndPoseMsgCounts.at(i)<<std::endl;
            }
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                count += robotBaseVelMsgCounts.at(i);
                std::cout<<robotBaseVelTopics.at(i)<<": "<<robotBaseVelMsgCounts.at(i)<<std::endl;
            }
            std::cout<<count<<std::endl;
            // for()
            // std::cout<<"cameraColorMsgCounts:"<<index<<" "<<cameraColorMsgCounts.at(index)<<std::endl;
            // std::cout<<"cameraDepthMsgCounts:"<<index<<" "<<cameraDepthMsgCounts.at(index)<<std::endl;
            // std::cout<<"cameraPointCloudMsgCounts:"<<index<<" "<<cameraPointCloudMsgCounts.at(index)<<std::endl;
            // std::cout<<"armJointStateMsgCounts:"<<index<<" "<<armJointStateMsgCounts.at(index)<<std::endl;
            // std::cout<<"armEndPoseMsgCounts:"<<index<<" "<<armEndPoseMsgCounts.at(index)<<std::endl;
            // std::cout<<"robotBaseVelMsgCounts:"<<index<<" "<<robotBaseVelMsgCounts.at(index)<<std::endl;
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_capture");
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    DataCapture dataCapture(datasetDir, episodeIndex);
    dataCapture.run();
    ROS_INFO("\033[1;32m----> data capture Started.\033[0m");
    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();
    dataCapture.join();
    std::cout<<"Done"<<std::endl;
    return 0;
}
