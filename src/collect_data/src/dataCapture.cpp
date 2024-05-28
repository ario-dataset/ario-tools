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
#include "jsoncpp/json/json.h"
#include "ryml.hpp"
#include "ryml_std.hpp"
#include "rapidjson/document.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/prettywriter.h"

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

    std::vector<BlockingDeque<sensor_msgs::Image>> cameraColorMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::Image>> cameraDepthMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::PointCloud2>> cameraPointCloudMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::JointState>> armJointStateMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::PoseStamped>> armEndPoseMsgDeques;
    std::vector<BlockingDeque<nav_msgs::Odometry>> robotBaseVelMsgDeques;

    std::vector<std::thread*> cameraColorSavingThreads;
    std::vector<std::thread*> cameraDepthSavingThreads;
    std::vector<std::thread*> cameraPointCloudSavingThreads;
    std::vector<std::thread*> armJointStateSavingThreads;
    std::vector<std::thread*> armEndPoseSavingThreads;
    std::vector<std::thread*> robotBaseVelSavingThreads;

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
        for(int i = 0; i < cameraColorNames.size(); i++){
            cameraColorSavingThreads.push_back(new std::thread(&DataCapture::cameraColorSaving, this, i));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            cameraDepthSavingThreads.push_back(new std::thread(&DataCapture::cameraDepthSaving, this, i));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            cameraPointCloudSavingThreads.push_back(new std::thread(&DataCapture::cameraPointCloudSaving, this, i));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            armJointStateSavingThreads.push_back(new std::thread(&DataCapture::armJointStateSaving, this, i));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            armEndPoseSavingThreads.push_back(new std::thread(&DataCapture::armEndPoseSaving, this, i));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            robotBaseVelSavingThreads.push_back(new std::thread(&DataCapture::robotBaseVelSaving, this, i));
        }
        keyboardInterruptCheckingThread = new std::thread(&DataCapture::keyboardInterruptChecking, this);
        monitoringThread = new std::thread(&DataCapture::monitoring, this);
    }
    /**
     * @brief join thread
     * 
     */
        void join(){
        for(int i = 0; i < cameraColorNames.size(); i++){
            cameraColorSavingThreads.at(i)->join();
            delete cameraColorSavingThreads.at(i);
            cameraColorSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            cameraDepthSavingThreads.at(i)->join();
            delete cameraDepthSavingThreads.at(i);
            cameraDepthSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            cameraPointCloudSavingThreads.at(i)->join();
            delete cameraPointCloudSavingThreads.at(i);
            cameraPointCloudSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            armJointStateSavingThreads.at(i)->join();
            delete armJointStateSavingThreads.at(i);
            armJointStateSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            armEndPoseSavingThreads.at(i)->join();
            delete armEndPoseSavingThreads.at(i);
            armEndPoseSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            robotBaseVelSavingThreads.at(i)->join();
            delete robotBaseVelSavingThreads.at(i);
            robotBaseVelSavingThreads.at(i) = nullptr;
        }
        keyboardInterruptCheckingThread->join();
        delete keyboardInterruptCheckingThread;
        keyboardInterruptCheckingThread = nullptr;
        monitoringThread->join();
        delete monitoringThread;
        monitoringThread = nullptr;
    }
    /**
     * @brief cameraColorHandler
     * 
     * @param msg 
     * @param index 
     */
    void cameraColorHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cameraColorMsgDeques.at(index).push_back(*msg);
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
        cameraDepthMsgDeques.at(index).push_back(*msg);
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
        cameraPointCloudMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraPointCloudMsgCountMtxs.at(index));
        cameraPointCloudMsgCounts.at(index) += 1;
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
		while(msg->header.frame_id != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(cameraColorParentFrames.at(index), msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraColorParentFrames.at(index), msg->header.frame_id, ros::Time(0), transform);
                break;
            } catch(tf::TransformException &ex) {
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

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->D.size(); i++){
            D.append(msg->D.at(i));
        }
        for(int i = 0; i < msg->K.size(); i++){
            K.append(msg->K.at(i));
        }
        for(int i = 0; i < msg->R.size(); i++){
            R.append(msg->R.at(i));
        }
        for(int i = 0; i < msg->P.size(); i++){
            P.append(msg->P.at(i));
        }
        root["D"] = D;
        root["K"] = K;
        root["R"] = R;
        root["P"] = P;
        root["binning_x"] = msg->binning_x;
        root["binning_y"] = msg->binning_y;
        root["roi"]["x_offset"] = msg->roi.x_offset;
        root["roi"]["y_offset"] = msg->roi.y_offset;
        root["roi"]["height"] = msg->roi.height;
        root["roi"]["width"] = msg->roi.width;
        root["roi"]["do_rectify"] = msg->roi.do_rectify;
        root["parent_frame"]["x"] = x;
        root["parent_frame"]["y"] = y;
        root["parent_frame"]["z"] = z;
        root["parent_frame"]["roll"] = roll;
        root["parent_frame"]["pitch"] = pitch;
        root["parent_frame"]["yaw"] = yaw;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(cameraColorDirs.at(index) + "/config.json");
        streamWriter.write(file, root);
        file.close();

        std::lock_guard<std::mutex> lock(cameraColorConfigMsgCountMtxs.at(index));
        cameraColorConfigMsgCounts.at(index) += 1;
        subCameraColorConfigs.at(index).shutdown();
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
		while(msg->header.frame_id != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(cameraDepthParentFrames.at(index), msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraDepthParentFrames.at(index), msg->header.frame_id, ros::Time(0), transform);
                break;
            } catch (tf::TransformException &ex) {
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

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->D.size(); i++){
            D.append(msg->D.at(i));
        }
        for(int i = 0; i < msg->K.size(); i++){
            K.append(msg->K.at(i));
        }
        for(int i = 0; i < msg->R.size(); i++){
            R.append(msg->R.at(i));
        }
        for(int i = 0; i < msg->P.size(); i++){
            P.append(msg->P.at(i));
        }
        root["D"] = D;
        root["K"] = K;
        root["R"] = R;
        root["P"] = P;
        root["binning_x"] = msg->binning_x;
        root["binning_y"] = msg->binning_y;
        root["roi"]["x_offset"] = msg->roi.x_offset;
        root["roi"]["y_offset"] = msg->roi.y_offset;
        root["roi"]["height"] = msg->roi.height;
        root["roi"]["width"] = msg->roi.width;
        root["roi"]["do_rectify"] = msg->roi.do_rectify;
        root["parent_frame"]["x"] = x;
        root["parent_frame"]["y"] = y;
        root["parent_frame"]["z"] = z;
        root["parent_frame"]["roll"] = roll;
        root["parent_frame"]["pitch"] = pitch;
        root["parent_frame"]["yaw"] = yaw;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(cameraDepthDirs.at(index) + "/config.json");
        streamWriter.write(file, root);
        file.close();

        std::lock_guard<std::mutex> lock(cameraDepthConfigMsgCountMtxs.at(index));
        cameraDepthConfigMsgCounts.at(index) += 1;
        subCameraDepthConfigs.at(index).shutdown();
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
		while(true && msg->header.frame_id != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(cameraPointCloudParentFrames.at(index), msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraPointCloudParentFrames.at(index), msg->header.frame_id, ros::Time(0), transform);
                break;
            } catch (tf::TransformException &ex) {
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

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->D.size(); i++){
            D.append(msg->D.at(i));
        }
        for(int i = 0; i < msg->K.size(); i++){
            K.append(msg->K.at(i));
        }
        for(int i = 0; i < msg->R.size(); i++){
            R.append(msg->R.at(i));
        }
        for(int i = 0; i < msg->P.size(); i++){
            P.append(msg->P.at(i));
        }
        root["D"] = D;
        root["K"] = K;
        root["R"] = R;
        root["P"] = P;
        root["binning_x"] = msg->binning_x;
        root["binning_y"] = msg->binning_y;
        root["roi"]["x_offset"] = msg->roi.x_offset;
        root["roi"]["y_offset"] = msg->roi.y_offset;
        root["roi"]["height"] = msg->roi.height;
        root["roi"]["width"] = msg->roi.width;
        root["roi"]["do_rectify"] = msg->roi.do_rectify;
        root["parent_frame"]["x"] = x;
        root["parent_frame"]["y"] = y;
        root["parent_frame"]["z"] = z;
        root["parent_frame"]["roll"] = roll;
        root["parent_frame"]["pitch"] = pitch;
        root["parent_frame"]["yaw"] = yaw;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(cameraPointCloudDirs.at(index) + "/config.json");
        streamWriter.write(file, root);
        file.close();

        std::lock_guard<std::mutex> lock(cameraPointCloudConfigMsgCountMtxs.at(index));
        cameraPointCloudConfigMsgCounts.at(index) += 1;
        subCameraPointCloudConfigs.at(index).shutdown();
    }
    /**
     * @brief armJointStateHandler
     * 
     * @param msg 
     * @param index 
     */
    void armJointStateHandler(const sensor_msgs::JointState::ConstPtr& msg, const int& index){
        armJointStateMsgDeques.at(index).push_back(*msg);
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
        armEndPoseMsgDeques.at(index).push_back(*msg);
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
        robotBaseVelMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(robotBaseVelMsgCountMtxs.at(index));
        robotBaseVelMsgCounts.at(index) += 1;
    }
    /**
     * @brief cameraColorSaving
     * 
     * @param index 
     */
    void cameraColorSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraColorMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::Image msg = cameraColorMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::imwrite(cameraColorDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".png", cv_ptr->image);
        }
    }
    /**
     * @brief cameraDepthSaving
     * 
     * @param index 
     */
    void cameraDepthSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraDepthMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::Image msg = cameraDepthMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::imwrite(cameraDepthDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".png", cv_ptr->image);
        }
    }
    /**
     * @brief cameraPointCloudSaving
     * 
     * @param index 
     */
    void cameraPointCloudSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraPointCloudMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::PointCloud2 msg = cameraPointCloudMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
            pcl::fromROSMsg(msg, pointcloud);
            pcl::io::savePCDFileBinary(cameraPointCloudDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".pcd", pointcloud);
        }
    }
    /**
     * @brief armJointStateSaving
     * 
     * @param index 
     */
    void armJointStateSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && armJointStateMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::JointState msg = armJointStateMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            Json::Value effort(Json::arrayValue);
            Json::Value position(Json::arrayValue);
            Json::Value velocity(Json::arrayValue);
            for(int i = 0; i < msg.effort.size(); i++){
                effort.append(msg.effort.at(i));
            }
            for(int i = 0; i < msg.position.size(); i++){
                position.append(msg.position.at(i));
            }
            for(int i = 0; i < msg.velocity.size(); i++){
                velocity.append(msg.velocity.at(i));
            }
            root["effort"] = effort;
            root["position"] = position;
            root["velocity"] = velocity;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(armJointStateDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }
    /**
     * @brief armEndPoseSaving
     * 
     * @param index 
     */
    void armEndPoseSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && armEndPoseMsgDeques.at(index).size() == 0)
                    break;
            }
            geometry_msgs::PoseStamped msg = armEndPoseMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            root["x"] = msg.pose.position.x;
            root["y"] = msg.pose.position.y;
            root["z"] = msg.pose.position.z;
            root["roll"] = msg.pose.orientation.x;
            root["pitch"] = msg.pose.orientation.y;
            root["yaw"] = msg.pose.orientation.z;
            root["grasper"] = msg.pose.orientation.w;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(armEndPoseDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }
    /**
     * @brief robotBaseVelSaving
     * 
     * @param index 
     */
    void robotBaseVelSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && robotBaseVelMsgDeques.at(index).size() == 0)
                    break;
            }
            nav_msgs::Odometry msg = robotBaseVelMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            root["linear"]["x"] = msg.twist.twist.linear.x;
            root["linear"]["y"] = msg.twist.twist.linear.y;
            root["angular"]["z"] = msg.twist.twist.angular.x;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(robotBaseVelDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }
    /**
     * @brief instructionSaving
     * 
     * @param instructions 
     */
    void instructionSaving(std::string instructions){
        if (instructions.front() != '[' || instructions.back() != ']') {
             std::cout << "Error parsing JSON: " << instructions << std::endl;
            return;
        }
        std::string content = instructions.substr(1, instructions.size() - 2);
        std::istringstream iss(content);
        std::string token;
        std::string jsonArray = "[";
        while (std::getline(iss, token, ',')) {
            if (!jsonArray.empty() && jsonArray.back() != '[') {
                jsonArray += ", ";
            }
            jsonArray += "\"" + token + "\"";
        }
        jsonArray += "]";

        Json::Value root, result;
        Json::CharReaderBuilder builder;
        Json::CharReader* reader = builder.newCharReader();

        std::string errors;
        bool parsingSuccessful = reader->parse(jsonArray.c_str(), jsonArray.c_str() + jsonArray.size(), &result, &errors);
        delete reader;

        if (!parsingSuccessful) {
            std::cout << "Error parsing JSON: " << jsonArray << std::endl;
            return;
        }

        root["instructions"] = result;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(instructionsDir);
        streamWriter.write(file, root);
        file.close();
    }
    /**
     * @brief keyboardInterruptChecking
     * 
     */
    void keyboardInterruptChecking(){
        std::string line;
        if (std::getline(std::cin, line)) {
            shutdown();
        }
    }
    /**
     * @brief shutdown
     * 
     */
    void shutdown(){
        captureStopMtx.lock();
        captureStop = true;
        captureStopMtx.unlock();
        for(int i = 0; i < cameraColorNames.size(); i++){
            subCameraColors.at(i).shutdown();
            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time().fromSec(0);
            cameraColorMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            subCameraDepths.at(i).shutdown();
            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time().fromSec(0);
            cameraDepthMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            subCameraPointClouds.at(i).shutdown();
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time().fromSec(0);
            cameraPointCloudMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            subArmJointStates.at(i).shutdown();
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time().fromSec(0);
            armJointStateMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            subArmEndPoses.at(i).shutdown();
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time().fromSec(0);
            armEndPoseMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            subRobotBaseVels.at(i).shutdown();
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time().fromSec(0);
            robotBaseVelMsgDeques.at(i).push_back(msg);
        }
        ros::shutdown();
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
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            system("clear");
            std::cout<<"path: "<<episodeDir<<std::endl;
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
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                cameraDepthConfigMsgCountMtxs.at(i).lock();
                int count = cameraDepthConfigMsgCounts.at(i);
                cameraDepthConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraDepthConfigTopics.at(i)<<": "<<count<<std::endl;
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                cameraPointCloudConfigMsgCountMtxs.at(i).lock();
                int count = cameraPointCloudConfigMsgCounts.at(i);
                cameraPointCloudConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraPointCloudConfigTopics.at(i)<<": "<<count<<std::endl;
            }
            std::cout<<std::endl;
            
            std::cout<<"press ENTER to stop capture:"<<std::endl;
            rate.sleep();
        }
    }
};


int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "data_capture");
    // 创建句柄
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    std::string instructions;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    nh.param<std::string>("instructions", instructions, "");
    ROS_INFO("\033[1;32m----> data capture Started.\033[0m");
    DataCapture dataCapture(datasetDir, episodeIndex);
    dataCapture.instructionSaving(instructions);
    dataCapture.run();
    ros::MultiThreadedSpinner spinner(16);
    spinner.spin();
    dataCapture.join();
    std::cout<<"Done"<<std::endl;
    return 0;
}
