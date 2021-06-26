#include <iostream>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/publisher.h"
#include <string>
//Data logger node
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//Ouster node
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>

//mkdir
#include <boost/filesystem.hpp>

//To write binary files
#include <fstream>

//Threading
#include <thread>

#define PREFIX_PATH "/mnt/DataDisk/"

#include <stdlib.h>

bool data_logging;
std::string data_prefix = "Stoplogging";
std::string stop_logging_msg = "Stoplogging";
std::string dir_name;
std::string dir_lidartest;

std::vector<std::thread> threads_test_point;
std::vector<bool> running_check_test_point;
int thread_test_point_num = 0;

std::vector<std::thread> threads_test_imu;
std::vector<bool> running_check_test_imu;
int thread_test_imu_num = 0;

void dataLoggingFlagCallback(const std_msgs::Bool::ConstPtr &msg){
    if(msg->data) {
        if(data_logging) {

        } else {
            data_logging = true;
            ROS_INFO("Data Logging Set True");
        }
    } else {
        if(data_logging){
            data_logging = false;
            ROS_INFO("Data Logging Set False");

            for(int i = 0; i < threads_test_point.size(); ++i) {
                if(threads_test_point[i].joinable()) {
                    threads_test_point[i].join();
                }
            }

            threads_test_point.clear();
            running_check_test_point.clear();

        }
    }
}

void dataPrefixCallBack(const std_msgs::String::ConstPtr & msg) {
    if(msg->data.compare(stop_logging_msg)!=0) {
        if(data_prefix.compare(stop_logging_msg) == 0) {
            std::cout<<"Prefix changed to logging"<<std::endl;
            data_prefix = msg->data;
            dir_name = PREFIX_PATH + data_prefix + "_lidar";
            dir_lidartest = dir_name + "/test";
            mkdir(dir_name.c_str(), 0777);
            mkdir(dir_lidartest.c_str(), 0777);
        }
    } else {
        if(data_prefix.compare(stop_logging_msg)!=0) {
            std::cout<<"Prefix changed to stop logging"<<std::endl;
            data_prefix = msg->data;
        }
    }
}

void save_pcd(sensor_msgs::PointCloud2 cloud_t,
              std::string data_dir, 
              int thread_no) {

    // data_dir = "/home/morin/test";

    std::stringstream ss;
    ss << data_dir << "/" << cloud_t.header.stamp << ".bin";

    std::string data_path = ss.str();

    std::ofstream binfile(data_path.c_str(), std::ios::out | std::ios::binary);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_t, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_t, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_t, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_t, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(cloud_t, "t");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_reflectivity(cloud_t, "reflectivity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ambient(cloud_t, "ambient");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_range(cloud_t, "range");

    while(iter_x!=iter_x.end()) {
        binfile.write((char*)&*iter_x, sizeof(float));
        binfile.write((char*)&*iter_y, sizeof(float));
        binfile.write((char*)&*iter_z, sizeof(float));
        binfile.write((char*)&*iter_intensity, sizeof(float));
        binfile.write((char*)&*iter_t, sizeof(uint32_t));
        binfile.write((char*)&*iter_reflectivity, sizeof(uint16_t));
        binfile.write((char*)&*iter_ambient, sizeof(uint16_t));
        binfile.write((char*)&*iter_range, sizeof(uint32_t));

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        ++iter_t;
        ++iter_reflectivity;
        ++iter_ambient;
        ++iter_range;
    }

    binfile.close();

    running_check_test_point[thread_no] = true;


    running_check_test_point[thread_no] = false;
}


void lidartest_handle(const sensor_msgs::PointCloud2::ConstPtr & cloud_msg){
    if(data_logging) {
        bool all_thread_running = true;
        int empty_thread = -1;
        for(int i = 0; i < thread_test_point_num; ++i) {
            if(running_check_test_point[i]){

            } else{
                empty_thread = i;
                all_thread_running = false;
                if(threads_test_point[i].joinable()){
                    threads_test_point[i].join();
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_t;
        cloud_t = *cloud_msg;

        if(all_thread_running) {
            running_check_test_point.push_back(false);
            threads_test_point.emplace_back(save_pcd, 
                                cloud_t, 
                                dir_lidartest,
                                running_check_test_point.size()-1);
            threads_test_point[running_check_test_point.size()-1].detach();
            ++thread_test_point_num;
        } else {
            threads_test_point[empty_thread] = std::thread(save_pcd, 
                                                *cloud_msg,
                                                dir_lidartest,
                                                empty_thread);
            threads_test_point[empty_thread].detach();    
        }

        int num_running_thread = 0;
        for(int i = 0; i < running_check_test_point.size(); ++i) {
            if(running_check_test_point[i]) {
                ++num_running_thread;
            }
        }

        std::stringstream ss;
        ss << dir_lidartest << "/" << cloud_t.header.stamp << ".bin";

        ROS_INFO("[LIDAR TEST] Data saved to %s", ss.str());
        ROS_INFO("[LIDAR TEST] Running Thread: %d", num_running_thread);

    } else {
        
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_ptcloud");

    ros::NodeHandle nh;
    ros::Subscriber sub_bool = nh.subscribe("/datalogging", 1, dataLoggingFlagCallback);
    ros::Subscriber sub_prefix = nh.subscribe("/save_prefix", 1, dataPrefixCallBack);
    ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_test/os_cloud_node/points", 1, lidartest_handle);
    std::cout<<"[POINTCLOUD SAVE NODE]"<<std::endl;
    std::cout<<"Subscribing to /lidar_test/os_cloud_node/points"<<std::endl;
    std::cout<<"Data save path set to:"<<PREFIX_PATH<<std::endl;

    ROS_INFO("Subscribing to /lidar_test/os_cloud_node/points");
    ROS_INFO("Data save path set to: %s", PREFIX_PATH);

    ros::spin();
 
    for(int i = 0; i<threads_test_point.size(); ++i) {
        threads_test_point[i].join();
    }

    return 0;
}