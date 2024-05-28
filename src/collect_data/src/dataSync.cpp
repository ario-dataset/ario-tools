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
// #include <filesystem>
#include <boost/filesystem.hpp>

class TimeSeries{
public:
    double time;
    std::vector<TimeSeries>* dataList;
    std::vector<TimeSeries>* syncList;

    TimeSeries(double time, std::vector<TimeSeries>* dataList, std::vector<TimeSeries>* syncList){
        this->dataList = dataList;
        this->syncList = syncList;
        this->time = time;
    }

    ~TimeSeries(){
        this->dataList = nullptr;
        this->syncList = nullptr;
    }

    void toDataList(){
        this->dataList->push_back(*this);
    }

    void toSyncList(){
        this->syncList->push_back(*this);
    }
};

class DataSync: public DataUtility{
public:
    std::vector<TimeSeries> allTimeSeries;

    std::vector<std::vector<TimeSeries>> cameraColorDataTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraDepthDataTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraPointCloudDataTimeSeries;
    std::vector<std::vector<TimeSeries>> armJointStateDataTimeSeries;
    std::vector<std::vector<TimeSeries>> armEndPoseDataTimeSeries;
    std::vector<std::vector<TimeSeries>> robotBaseVelDataTimeSeries;

    std::vector<std::vector<TimeSeries>> cameraColorSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraDepthSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraPointCloudSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> armJointStateSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> armEndPoseSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> robotBaseVelSyncTimeSeries;

    DataSync(std::string datasetDir, int episodeIndex): DataUtility(datasetDir, episodeIndex) {
        cameraColorDataTimeSeries = std::vector<std::vector<TimeSeries>>(cameraColorNames.size());
        cameraDepthDataTimeSeries = std::vector<std::vector<TimeSeries>>(cameraDepthNames.size());
        cameraPointCloudDataTimeSeries = std::vector<std::vector<TimeSeries>>(cameraPointCloudNames.size());
        armJointStateDataTimeSeries = std::vector<std::vector<TimeSeries>>(armJointStateNames.size());
        armEndPoseDataTimeSeries = std::vector<std::vector<TimeSeries>>(armEndPoseNames.size());
        robotBaseVelDataTimeSeries = std::vector<std::vector<TimeSeries>>(robotBaseVelNames.size());

        cameraColorSyncTimeSeries = std::vector<std::vector<TimeSeries>>(cameraColorNames.size());
        cameraDepthSyncTimeSeries = std::vector<std::vector<TimeSeries>>(cameraDepthNames.size());
        cameraPointCloudSyncTimeSeries = std::vector<std::vector<TimeSeries>>(cameraPointCloudNames.size());
        armJointStateSyncTimeSeries = std::vector<std::vector<TimeSeries>>(armJointStateNames.size());
        armEndPoseSyncTimeSeries = std::vector<std::vector<TimeSeries>>(armEndPoseNames.size());
        robotBaseVelSyncTimeSeries = std::vector<std::vector<TimeSeries>>(robotBaseVelNames.size());

        for(int i = 0; i < cameraColorNames.size(); i++){
            getFileInPath(cameraColorDirs.at(i), ".png", &cameraColorDataTimeSeries.at(i), &cameraColorSyncTimeSeries.at(i));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            getFileInPath(cameraDepthDirs.at(i), ".png", &cameraDepthDataTimeSeries.at(i), &cameraDepthSyncTimeSeries.at(i));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            getFileInPath(cameraPointCloudDirs.at(i), ".pcd", &cameraPointCloudDataTimeSeries.at(i), &cameraPointCloudSyncTimeSeries.at(i));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            getFileInPath(armJointStateDirs.at(i), ".json", &armJointStateDataTimeSeries.at(i), &armJointStateSyncTimeSeries.at(i));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            getFileInPath(armEndPoseDirs.at(i), ".json", &armEndPoseDataTimeSeries.at(i), &armEndPoseSyncTimeSeries.at(i));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            getFileInPath(robotBaseVelDirs.at(i), ".json", &robotBaseVelDataTimeSeries.at(i), &robotBaseVelSyncTimeSeries.at(i));
        }
        std::sort(allTimeSeries.begin(), allTimeSeries.end(), [](const TimeSeries& a, const TimeSeries& b){
            return a.time < b.time;
        });
    }

    void sync(){
        int frameCount = 0;
        std::cout<<"allTimeSeries:"<<allTimeSeries.size()<<std::endl;
        for(int i = 0; i < allTimeSeries.size(); i++){
            allTimeSeries.at(i).toDataList();
            double frameTime = checkDataAdequacy();
            if(frameTime != INFINITY){
                // frameTime = allTimeSeries.at(i).time;
                for(int i = 0; i < cameraColorNames.size(); i++){
                    if(!cameraColorToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < cameraColorDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(cameraColorDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    cameraColorDataTimeSeries.at(i).at(closerIndex).toSyncList();
                    cameraColorDataTimeSeries.at(i).erase(cameraColorDataTimeSeries.at(i).begin(), cameraColorDataTimeSeries.at(i).begin() + closerIndex + 1);
                }
                for(int i = 0; i < cameraDepthNames.size(); i++){
                    if(!cameraDepthToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < cameraDepthDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(cameraDepthDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    cameraDepthDataTimeSeries.at(i).at(closerIndex).toSyncList();
                    cameraDepthDataTimeSeries.at(i).erase(cameraDepthDataTimeSeries.at(i).begin(), cameraDepthDataTimeSeries.at(i).begin() + closerIndex + 1);
                }
                for(int i = 0; i < cameraPointCloudNames.size(); i++){
                    if(!cameraPointCloudToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < cameraPointCloudDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(cameraPointCloudDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    cameraPointCloudDataTimeSeries.at(i).at(closerIndex).toSyncList();
                    cameraPointCloudDataTimeSeries.at(i).erase(cameraPointCloudDataTimeSeries.at(i).begin(), cameraPointCloudDataTimeSeries.at(i).begin() + closerIndex + 1);
                }
                for(int i = 0; i < armJointStateNames.size(); i++){
                    if(!armJointStateToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < armJointStateDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(armJointStateDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    armJointStateDataTimeSeries.at(i).at(closerIndex).toSyncList();
                    armJointStateDataTimeSeries.at(i).erase(armJointStateDataTimeSeries.at(i).begin(), armJointStateDataTimeSeries.at(i).begin() + closerIndex + 1);
                }
                for(int i = 0; i < armEndPoseNames.size(); i++){
                    if(!armEndPoseToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < armEndPoseDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(armEndPoseDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    armEndPoseDataTimeSeries.at(i).at(closerIndex).toSyncList();
                    armEndPoseDataTimeSeries.at(i).erase(armEndPoseDataTimeSeries.at(i).begin(), armEndPoseDataTimeSeries.at(i).begin() + closerIndex + 1);
                }
                for(int i = 0; i < robotBaseVelNames.size(); i++){
                    if(!robotBaseVelToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < robotBaseVelDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(robotBaseVelDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    robotBaseVelDataTimeSeries.at(i).at(closerIndex).toSyncList();
                    robotBaseVelDataTimeSeries.at(i).erase(robotBaseVelDataTimeSeries.at(i).begin(), robotBaseVelDataTimeSeries.at(i).begin() + closerIndex + 1);
                }
                frameCount += 1;
            }
        }
        std::cout<<"sync frame num:"<<frameCount<<std::endl;
        for(int i = 0; i < cameraColorNames.size(); i++){
            std::ofstream file(cameraColorDirs.at(i) + "/sync.txt");
            for(int j = 0; j < cameraColorSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(cameraColorSyncTimeSeries.at(i).at(j).time)<<".png"<<std::endl;
            file.close();
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            std::ofstream file(cameraDepthDirs.at(i) + "/sync.txt");
            for(int j = 0; j < cameraDepthSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(cameraDepthSyncTimeSeries.at(i).at(j).time)<<".png"<<std::endl;
            file.close();
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            std::ofstream file(cameraPointCloudDirs.at(i) + "/sync.txt");
            for(int j = 0; j < cameraPointCloudSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(cameraPointCloudSyncTimeSeries.at(i).at(j).time)<<".pcd"<<std::endl;
            file.close();
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            std::ofstream file(armJointStateDirs.at(i) + "/sync.txt");
            for(int j = 0; j < armJointStateSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(armJointStateSyncTimeSeries.at(i).at(j).time)<<".json"<<std::endl;
            file.close();
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            std::ofstream file(armEndPoseDirs.at(i) + "/sync.txt");
            for(int j = 0; j < armEndPoseSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(armEndPoseSyncTimeSeries.at(i).at(j).time)<<".json"<<std::endl;
            file.close();
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            std::ofstream file(robotBaseVelDirs.at(i) + "/sync.txt");
            for(int j = 0; j < robotBaseVelSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(robotBaseVelSyncTimeSeries.at(i).at(j).time)<<".json"<<std::endl;
            file.close();
        }
    }

    double checkDataAdequacy(){
        bool result = true;
        double time = INFINITY;
        for(int i = 0; i < cameraColorNames.size() && result; i++){
            if(cameraColorToSyncs.at(i)){
                if(cameraColorDataTimeSeries.at(i).size() == 0)
                    result = false;
                else
                    time = time < cameraColorDataTimeSeries.at(i).back().time ? time : cameraColorDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < cameraDepthNames.size() && result; i++){
            if(cameraDepthToSyncs.at(i)){
                if(cameraDepthDataTimeSeries.at(i).size() == 0)
                    result = false;
                else
                    time = time < cameraDepthDataTimeSeries.at(i).back().time ? time : cameraDepthDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < cameraPointCloudNames.size() && result; i++){
            if(cameraPointCloudToSyncs.at(i)){
                if(cameraPointCloudDataTimeSeries.at(i).size() == 0)
                    result = false;
                else
                    time = time < cameraPointCloudDataTimeSeries.at(i).back().time ? time : cameraPointCloudDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < armJointStateNames.size() && result; i++){
            if(armJointStateToSyncs.at(i)){
                if(armJointStateDataTimeSeries.at(i).size() == 0)
                    result = false;
                else
                    time = time < armJointStateDataTimeSeries.at(i).back().time ? time : armJointStateDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < armEndPoseNames.size() && result; i++){
            if(armEndPoseToSyncs.at(i)){
                if(armEndPoseDataTimeSeries.at(i).size() == 0)
                    result = false;
                else
                    time = time < armEndPoseDataTimeSeries.at(i).back().time ? time : armEndPoseDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < robotBaseVelNames.size() && result; i++){
            if(robotBaseVelToSyncs.at(i)){
                if(robotBaseVelDataTimeSeries.at(i).size() == 0)
                    result = false;
                else
                    time = time < robotBaseVelDataTimeSeries.at(i).back().time ? time : robotBaseVelDataTimeSeries.at(i).back().time;
            }
        }
        return result ? time : INFINITY;
    }

    void getFileInPath(std::string path, std::string ext, std::vector<TimeSeries>* dataList, std::vector<TimeSeries>* syncList){
        for (const auto& entry : boost::filesystem::directory_iterator(path)) {
            const auto& path = entry.path();
            if (path.extension() == ext) {
                try{
                    allTimeSeries.push_back(TimeSeries(std::stod(path.stem().string()), dataList, syncList));
                }catch(std::invalid_argument &ex){
                    continue;
                }
            }
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_sync");
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    ROS_INFO("\033[1;32m----> data sync Started.\033[0m");
    if(episodeIndex == -1){
        for (const auto& entry : boost::filesystem::directory_iterator(datasetDir)) {
            const auto& path = entry.path();
            std::string fileName = path.stem().string();
            fileName.replace(0, 7, "");
            DataSync dataSync(datasetDir, std::stoi(fileName));
            dataSync.sync();
        }
    }else{
        DataSync dataSync(datasetDir, episodeIndex);
        dataSync.sync();
    }
    std::cout<<"Done"<<std::endl;
    return 0;
}
