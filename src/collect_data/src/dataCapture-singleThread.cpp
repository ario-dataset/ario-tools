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
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

/**
 * @brief 继承自DataUtility
 * 
 * @param std::string datasetDir, 
 * @param int episodeIndex
 */
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

    std::vector<int> cameraColorMsgCounts;
    std::vector<int> cameraDepthMsgCounts;
    std::vector<int> cameraPointCloudMsgCounts;
    std::vector<int> armJointStateMsgCounts;
    std::vector<int> armEndPoseMsgCounts;
    std::vector<int> robotBaseVelMsgCounts;

    std::vector<int> cameraColorConfigMsgCounts;
    std::vector<int> cameraDepthConfigMsgCounts;
    std::vector<int> cameraPointCloudConfigMsgCounts;
    std::vector<int> armJointStateConfigMsgCounts;
    std::vector<int> armEndPoseConfigMsgCounts;
    std::vector<int> robotBaseVelConfigMsgCounts;
    
    std::vector<std::mutex> cameraColorMsgCountMtxs;
    std::vector<std::mutex> cameraDepthMsgCountMtxs;
    std::vector<std::mutex> cameraPointCloudMsgCountMtxs;
    std::vector<std::mutex> armJointStateMsgCountMtxs;
    std::vector<std::mutex> armEndPoseMsgCountMtxs;
    std::vector<std::mutex> robotBaseVelMsgCountMtxs;

    std::vector<std::mutex> cameraColorConfigMsgCountMtxs;
    std::vector<std::mutex> cameraDepthConfigMsgCountMtxs;
    std::vector<std::mutex> cameraPointCloudConfigMsgCountMtxs;
    std::vector<std::mutex> armJointStateConfigMsgCountMtxs;
    std::vector<std::mutex> armEndPoseConfigMsgCountMtxs;
    std::vector<std::mutex> robotBaseVelConfigMsgCountMtxs;

    std::thread* keyboardInterruptCheckingThread;
    std::thread* monitoringThread;
    bool keyboardInterrupt = false;
    bool captureStop = false;
    std::mutex captureStopMtx;
    /**
     * @brief Construct a new Data Capture object
     * 
     * @param datasetDir 
     * @param episodeIndex 
     */
    DataCapture(std::string datasetDir, int episodeIndex): DataUtility(datasetDir, episodeIndex) {
        int unused = system((std::string("rm -r ") + episodeDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraColorDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraDepthDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraPointCloudDir).c_str());
        unused = system((std::string("mkdir -p ") + armJointStateDir).c_str());
        unused = system((std::string("mkdir -p ") + armEndPoseDir).c_str());
        unused = system((std::string("mkdir -p ") + robotBaseVelDir).c_str());

        cameraColorMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        robotBaseVelMsgCounts = std::vector<int>(robotBaseVelNames.size(), 0);

        cameraColorConfigMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthConfigMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudConfigMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateConfigMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseConfigMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        robotBaseVelConfigMsgCounts = std::vector<int>(robotBaseVelNames.size(), 0);

        cameraColorMsgCountMtxs = std::vector<std::mutex>(cameraColorNames.size());
        cameraDepthMsgCountMtxs = std::vector<std::mutex>(cameraDepthNames.size());
        cameraPointCloudMsgCountMtxs = std::vector<std::mutex>(cameraPointCloudNames.size());
        armJointStateMsgCountMtxs = std::vector<std::mutex>(armJointStateNames.size());
        armEndPoseMsgCountMtxs = std::vector<std::mutex>(armEndPoseNames.size());
        robotBaseVelMsgCountMtxs = std::vector<std::mutex>(robotBaseVelNames.size());

        cameraColorConfigMsgCountMtxs = std::vector<std::mutex>(cameraColorNames.size());
        cameraDepthConfigMsgCountMtxs = std::vector<std::mutex>(cameraDepthNames.size());
        cameraPointCloudConfigMsgCountMtxs = std::vector<std::mutex>(cameraPointCloudNames.size());
        armJointStateConfigMsgCountMtxs = std::vector<std::mutex>(armJointStateNames.size());
        armEndPoseConfigMsgCountMtxs = std::vector<std::mutex>(armEndPoseNames.size());
        robotBaseVelConfigMsgCountMtxs = std::vector<std::mutex>(robotBaseVelNames.size());

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
    /**
     * @brief run thread
     * 
     */
    void run(){
        keyboardInterruptCheckingThread = new std::thread(&DataCapture::keyboardInterruptChecking, this);
        monitoringThread = new std::thread(&DataCapture::monitoring, this);
    }
    /**
     * @brief join thread
     * 
     */
    void join(){
        keyboardInterruptCheckingThread->join();
        delete keyboardInterruptCheckingThread;
        monitoringThread->join();
        delete monitoringThread;
    }
    /**
     * @brief cameraColorHandler
     * 
     * @param msg 
     * @param index 
     */
    void cameraColorHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        cv::imwrite(cameraColorDirs.at(index) + "/" + std::to_string(msg->header.stamp.toSec()) + ".png", cv_ptr->image);
        std::lock_guard<std::mutex> lock(cameraColorMsgCountMtxs.at(index));
        cameraColorMsgCounts.at(index) += 1;
    }
    /**
     * @brief cameraDepthHandler
     * 
     * @param msg 
     * @param index 
     */
    void cameraDepthHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::imwrite(cameraDepthDirs.at(index) + "/" + std::to_string(msg->header.stamp.toSec()) + ".png", cv_ptr->image);
        std::lock_guard<std::mutex> lock(cameraDepthMsgCountMtxs.at(index));
        cameraDepthMsgCounts.at(index) += 1;
    }
    /**
     * @brief cameraPointCloudHandler
     * 
     * @param msg 
     * @param index 
     */
    void cameraPointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& msg, const int& index){
        // static std::chrono::time_point<std::chrono::system_clock> time_start, time_end;
        // double exe_time;
        // time_start = std::chrono::system_clock::now();
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
        pcl::fromROSMsg(*msg, pointcloud);
        pcl::io::savePCDFileBinary(cameraPointCloudDirs.at(index) + "/" + std::to_string(msg->header.stamp.toSec()) + ".pcd", pointcloud);
        std::lock_guard<std::mutex> lock(cameraPointCloudMsgCountMtxs.at(index));
        cameraPointCloudMsgCounts.at(index) += 1;
        // time_end = std::chrono::system_clock::now();
        // exe_time = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count() / 1000.0;
    }
    /**
     * @brief cameraColorConfigHandler
     * 
     * @param msg 
     * @param index 
     */
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
        std::lock_guard<std::mutex> lock(cameraColorConfigMsgCountMtxs.at(index));
        cameraColorConfigMsgCounts.at(index) += 1;
    }
    /**
     * @brief cameraDepthConfigHandler
     * 
     * @param msg 
     * @param index 
     */
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
        std::lock_guard<std::mutex> lock(cameraDepthConfigMsgCountMtxs.at(index));
        cameraDepthConfigMsgCounts.at(index) += 1;
    }
    /**
     * @brief cameraPointCloudConfigHandler
     * 
     * @param msg 
     * @param index 
     */
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
        std::lock_guard<std::mutex> lock(cameraPointCloudConfigMsgCountMtxs.at(index));
        cameraPointCloudConfigMsgCounts.at(index) += 1;
    }
    /**
     * @brief armJointStateHandler
     * 
     * @param msg 
     * @param index 
     */
    void armJointStateHandler(const sensor_msgs::JointState::ConstPtr& msg, const int& index){
        std::ofstream file(armJointStateDirs.at(index) + "/" + std::to_string(msg->header.stamp.toSec()) + ".txt");
        file<<"effort: [";
        for(int j = 0; j < msg->effort.size(); j++){
            if(j != 0)
                file<<", ";
            file<<msg->effort.at(j);
        }
        file<<"]"<<std::endl;
        file<<"position: [";
        for(int j = 0; j < msg->position.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->position.at(j);
        }
        file<<"]"<<std::endl;
        file<<std::endl;
        file<<"velocity: [";
        for(int j = 0; j < msg->velocity.size(); j++){
            if(j != 0)
                file<<", ";
            file<<" "<<msg->velocity.at(j);
        }
        file<<"]"<<std::endl;
        file.close();
        std::lock_guard<std::mutex> lock(armJointStateMsgCountMtxs.at(index));
        armJointStateMsgCounts.at(index) += 1;
    }
    /**
     * @brief armEndPoseHandler
     * 
     * @param msg 
     * @param index 
     */
    void armEndPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg, const int& index){
        std::ofstream file(armEndPoseDirs.at(index) + "/" + std::to_string(msg->header.stamp.toSec()) + ".txt");
        file<<"x: "<<msg->pose.position.x<<std::endl;
        file<<"y: "<<msg->pose.position.y<<std::endl;
        file<<"z: "<<msg->pose.position.z<<std::endl;
        file<<"roll: "<<msg->pose.orientation.x<<std::endl;
        file<<"pitch: "<<msg->pose.orientation.y<<std::endl;
        file<<"yaw: "<<msg->pose.orientation.z<<std::endl;
        file<<"grasper: "<<msg->pose.orientation.w<<std::endl;
        file.close();
        std::lock_guard<std::mutex> lock(armEndPoseMsgCountMtxs.at(index));
        armEndPoseMsgCounts.at(index) += 1;
    }
    /**
     * @brief robotBaseVelHandler
     * 
     * @param msg 
     * @param index 
     */
    void robotBaseVelHandler(const nav_msgs::Odometry::ConstPtr& msg, const int& index){
        std::ofstream file(robotBaseVelDirs.at(index) + "/" + std::to_string(msg->header.stamp.toSec()) + ".txt");
        file<<"linear_x: "<<msg->twist.twist.linear.x<<std::endl;
        file<<"linear_y: "<<msg->twist.twist.linear.y<<std::endl;
        file<<"angular_z: "<<msg->twist.twist.angular.x<<std::endl;
        file.close();
        std::lock_guard<std::mutex> lock(robotBaseVelMsgCountMtxs.at(index));
        robotBaseVelMsgCounts.at(index) += 1;
    }
    /**
     * @brief keyboardInterruptChecking
     * 
     */
    void keyboardInterruptChecking(){
        std::string line;
        if (std::getline(std::cin, line)) {
            ros::shutdown();
        }
    }
    /**
     * @brief monitoring
     * 
     */
    void monitoring(){
        ros::Rate rate = ros::Rate(1);
        std::vector<int> cameraColorMsgLastCounts = std::vector<int>(cameraColorNames.size(), 0);
        std::vector<int> cameraDepthMsgLastCounts = std::vector<int>(cameraDepthNames.size(), 0);
        std::vector<int> cameraPointCloudMsgLastCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        std::vector<int> armJointStateMsgLastCounts = std::vector<int>(armJointStateNames.size(), 0);
        std::vector<int> armEndPoseMsgLastCounts = std::vector<int>(armEndPoseNames.size(), 0);
        std::vector<int> robotBaseVelMsgLastCounts = std::vector<int>(robotBaseVelNames.size(), 0);
        ros::Time beginTime = ros::Time::now();
        while(ros::ok()){
            system("clear");
            std::cout<<"total time: "<<(ros::Time::now() - beginTime).toSec()<<std::endl;
            int allCount = 0;
            std::cout<<"topic: frame in 1 second / total frame"<<std::endl;
            for(int i = 0; i < cameraColorNames.size(); i++){
                cameraColorMsgCountMtxs.at(i).lock();
                int count = cameraColorMsgCounts.at(i);
                cameraColorMsgCountMtxs.at(i).unlock();
                std::cout<<cameraColorTopics.at(i)<<": "<<(count - cameraColorMsgLastCounts.at(i))<<" / "<<count<<std::endl;
                cameraColorMsgLastCounts.at(i) = count;
                allCount += count;
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                cameraDepthMsgCountMtxs.at(i).lock();
                int count = cameraDepthMsgCounts.at(i);
                cameraDepthMsgCountMtxs.at(i).unlock();
                std::cout<<cameraDepthTopics.at(i)<<": "<<(count - cameraDepthMsgLastCounts.at(i))<<" / "<<count<<std::endl;
                cameraDepthMsgLastCounts.at(i) = count;
                allCount += count;
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                cameraPointCloudMsgCountMtxs.at(i).lock();
                int count = cameraPointCloudMsgCounts.at(i);
                cameraPointCloudMsgCountMtxs.at(i).unlock();
                std::cout<<cameraPointCloudTopics.at(i)<<": "<<(count - cameraPointCloudMsgLastCounts.at(i))<<" / "<<count<<std::endl;
                cameraPointCloudMsgLastCounts.at(i) = count;
                allCount += count;
            }
            for(int i = 0; i < armJointStateNames.size(); i++){
                armJointStateMsgCountMtxs.at(i).lock();
                int count = armJointStateMsgCounts.at(i);
                armJointStateMsgCountMtxs.at(i).unlock();
                std::cout<<armJointStateTopics.at(i)<<": "<<(count - armJointStateMsgLastCounts.at(i))<<" / "<<count<<std::endl;
                armJointStateMsgLastCounts.at(i) = count;
                allCount += count;
            }
            for(int i = 0; i < armEndPoseNames.size(); i++){
                armEndPoseMsgCountMtxs.at(i).lock();
                int count = armEndPoseMsgCounts.at(i);
                armEndPoseMsgCountMtxs.at(i).unlock();
                std::cout<<armEndPoseTopics.at(i)<<": "<<(count - armEndPoseMsgLastCounts.at(i))<<" / "<<count<<std::endl;
                armEndPoseMsgLastCounts.at(i) = count;
                allCount += count;
            }
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                robotBaseVelMsgCountMtxs.at(i).lock();
                int count = robotBaseVelMsgCounts.at(i);
                robotBaseVelMsgCountMtxs.at(i).unlock();
                std::cout<<robotBaseVelTopics.at(i)<<": "<<(count - robotBaseVelMsgLastCounts.at(i))<<" / "<<count<<std::endl;
                robotBaseVelMsgLastCounts.at(i) = count;
                allCount += count;
            }
            std::cout<<"sum total frame: "<<allCount<<std::endl;
            std::cout<<std::endl;

            std::cout<<"config topic: total frame"<<std::endl;
            for(int i = 0; i < cameraColorNames.size(); i++){
                cameraColorConfigMsgCountMtxs.at(i).lock();
                int count = cameraColorConfigMsgCounts.at(i);
                cameraColorConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraColorConfigTopics.at(i)<<": "<<count<<std::endl;
                if(count != 0)
                    subCameraColorConfigs.at(i).shutdown();
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                cameraDepthConfigMsgCountMtxs.at(i).lock();
                int count = cameraDepthConfigMsgCounts.at(i);
                cameraDepthConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraDepthConfigTopics.at(i)<<": "<<count<<std::endl;
                if(count != 0)
                    subCameraDepthConfigs.at(i).shutdown();
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                cameraPointCloudConfigMsgCountMtxs.at(i).lock();
                int count = cameraPointCloudConfigMsgCounts.at(i);
                cameraPointCloudConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraPointCloudConfigTopics.at(i)<<": "<<count<<std::endl;
                if(count != 0)
                    subCameraPointCloudConfigs.at(i).shutdown();
            }
            std::cout<<std::endl;
            
            std::cout<<"press ENTER to stop capture:"<<std::endl;
            rate.sleep();
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
