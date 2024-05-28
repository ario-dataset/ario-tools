/* Copyright 2024 Agilex Robotics Inc. All Rights Reserved.

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
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
// #include <filesystem>
#include <boost/filesystem.hpp>

class PointCloudFilter: public DataUtility{
public:

    std::vector<std::string> cameraPointCloudNormalizationDirs;

    PointCloudFilter(std::string datasetDir, int episodeIndex): DataUtility(datasetDir, episodeIndex) {
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            cameraPointCloudNormalizationDirs.push_back(cameraPointCloudDirs.at(i) + "-normalization");
            int unused = system((std::string("mkdir -p ") + cameraPointCloudNormalizationDirs.at(i)).c_str());
        }
    }

    void filter(){
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            for (const auto& entry : boost::filesystem::directory_iterator(cameraPointCloudDirs.at(i))) {
                const auto& path = entry.path();
                if (path.extension() == ".pcd") {
                    
                }else{
                    boost::filesystem::path dstPath = boost::filesystem::path(cameraPointCloudNormalizationDirs.at(i)) / path.filename();
                    std::ifstream srcFile(path.string(), std::ios::binary);
                    std::ofstream dstFile(dstPath.string(), std::ios::binary);
                    dstFile << srcFile.rdbuf();
                    srcFile.close();
                    dstFile.close();
                }
            }
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            std::ifstream syncFile;
            syncFile.open(cameraPointCloudDirs.at(i)+"/sync.txt");
            while(true){
                std::string time;
                if(!getline(syncFile, time))
                    break;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudTmp(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudNorm(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::io::loadPCDFile<pcl::PointXYZRGB>(cameraPointCloudDirs.at(i) + "/" + time, *pointCloud);
                pcl::copyPointCloud(*pointCloud, *pointCloudTmp);
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(pointCloudTmp);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0, 2);
                pass.setFilterLimitsNegative(false);
                pass.filter(*pointCloudNorm);
                pcl::io::savePCDFileBinary(cameraPointCloudNormalizationDirs.at(i) + "/" + time, *pointCloudNorm);
            }
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    ROS_INFO("\033[1;32m----> pointcloud filter Started.\033[0m");
    if(episodeIndex == -1){
        for (const auto& entry : boost::filesystem::directory_iterator(datasetDir)) {
            const auto& path = entry.path();
            std::string fileName = path.stem().string();
            fileName.replace(0, 7, "");
            PointCloudFilter pointCloudFilter(datasetDir, std::stoi(fileName));
            pointCloudFilter.filter();
        }
    }else{
        PointCloudFilter pointCloudFilter(datasetDir, episodeIndex);
        pointCloudFilter.filter();
    }
    std::cout<<"Done"<<std::endl;
    return 0;
}
