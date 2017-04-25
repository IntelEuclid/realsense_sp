/******************************************************************************
 Copyright (c) 2016, Intel Corporation
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once
#ifndef PERSON_NODELET
#define PERSON_NODELET

#include <stdlib.h>
#include <mutex>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <librealsense/slam/slam.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include <rs_sdk.h>

#include <realsense_camera/GetIMUInfo.h>
#include <realsense_sp/Status.h>
#include <realsense_sp/Reset.h>
#include <realsense_camera/SamplingData.h>
//namespace RSCore = rs::core;

namespace realsense_sp
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> camInfoSyncer;
typedef message_filters::TimeSynchronizer<sensor_msgs::Image, realsense_camera::SamplingData> imageSamplingSyncer;

class slam_tracking_event_handler : public rs::slam::tracking_event_handler
{
public:
  slam_tracking_event_handler() { 
  }

    
  void on_restart()
  {
    ROS_INFO("Restart--------------------------------");
    fflush(stdout);
  }
  ~slam_tracking_event_handler()
  {
    ROS_INFO("deconstructure");
  }
};
class slam_event_handler : public rs::core::video_module_interface::processing_event_handler
{
public:
  ros::NodeHandle nodeHandle_;
  ros::Publisher odomPublisher;
  ros::Publisher statusPublisher;;
  ros::Publisher occPublisher;
  
  tf::TransformBroadcaster br;
  tf::Transform transform;
  std::shared_ptr<rs::slam::occupancy_map> occ_map;
  std::string camera_frame_id_;
  std::string origin_frame_id_;
  geometry_msgs::Pose previousPose;
  ros::Time previousTimestamp = ros::Time(0);
  slam_event_handler(ros::NodeHandle &n) : nodeHandle_(n)
  {
    std::cout << "created.........." << std::endl;
    odomPublisher = n.advertise<nav_msgs::Odometry>("/realsense/odom",1);  
    statusPublisher = n.advertise<realsense_sp::Status>("/realsense/slam/status",1);  
    
    n.param("camera_frame_id", camera_frame_id_, std::string("camera_link"));
    n.param("origin_frame_id", origin_frame_id_, std::string("base_link"));
    //occPublisher = n.advertise<nav_msgs::OccupancyGrid>("/map",1);  
  }
  void module_output_ready(rs::core::video_module_interface *sender, rs::core::correlated_sample_set *sample);

  ~slam_event_handler()
  {
    ROS_INFO("deconstructure event handler");
  }
};

class SPNodelet : public nodelet::Nodelet
{
public:
  SPNodelet();
  ~SPNodelet();
  void onInit();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string nodelet_name_;
  ros::ServiceServer resetClient;
  ros::ServiceClient imu_info_client_;
  std::unique_ptr<rs::slam::slam> slam_;
  slam_event_handler *scenePerceptionEventHandler;
  slam_tracking_event_handler trackingEventHandler;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> fisheye_caminfo_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> depth_caminfo_sub_;
  std::shared_ptr<message_filters::Synchronizer<camInfoSyncer>> caminfo_tsync_;
  message_filters::Connection caminfo_connection_;
  ros::Subscriber imu_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>  fisheye_sub_, depth_sub_;
  std::shared_ptr<message_filters::Subscriber<realsense_camera::SamplingData>>  fisheye_sampling_sub_, depth_sampling_sub_;
  bool enable_relocalization_;
  bool ready_;


  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, realsense_camera::SamplingData>> fisheye_sync_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, realsense_camera::SamplingData>> depth_sync_;
  message_filters::Connection image_connection_;
  rs::core::video_module_interface::supported_module_config supported_config;
  rs::core::video_module_interface::actual_module_config actual_config;

  int64_t fisheyeCount = 0;
  int64_t gyroCount = 0;
  int64_t accelCount = 0;
  int64_t depthCount = 0;

  void getStaticParameters();
  void subscribeToCamInfoTopics();
  void caminfoCallback(const sensor_msgs::CameraInfoConstPtr &color_caminfo,
                       const sensor_msgs::CameraInfoConstPtr &depth_caminfo);
  void initializeIntrinsicData(const sensor_msgs::CameraInfoConstPtr &cameraInfoMsg, rs::core::intrinsics &intrinsic);
  void initializeIMUIntrinsicData(realsense_camera::IMUInfo &imu_res, rs::core::motion_device_intrinsics &motion_intrin);
  void initializeExtrinsicData(realsense_camera::GetIMUInfo &extrinsic, rs::core::extrinsics &fe2imu, rs::core::extrinsics &depth2fe);
  void initializeSlam();
  void getParameters();
  void subscribeToCameraStreams();
  void depthMessageCallback(const sensor_msgs::ImageConstPtr &depthImageMsg, const realsense_camera::SamplingDataConstPtr &sampling_data);
  void fisheyeMessageCallback(const sensor_msgs::ImageConstPtr &fisheyeImageMsg, const realsense_camera::SamplingDataConstPtr &sampling_data);
  void imuMessageCallback(const sensor_msgs::ImuConstPtr &imuDataMsg);
  void printConfig(rs::core::video_module_interface::actual_module_config &slam_config);
  rs::core::image_interface *rosImageToRSImage(const sensor_msgs::ImageConstPtr &image, rs::core::stream_type stream,const realsense_camera::SamplingDataConstPtr& sampling_data );
  bool reset(realsense_sp::Reset::Request & req,realsense_sp::Reset::Response & res);
  std::mutex mut_depth, mut_fisheye, mut_imu;
  // void getImageIntrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo, RSCore::intrinsics& intrinsics);
  //void getImageExtrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo, RSCore::extrinsics& extrinsics);
  // void setModuleConfig(int image_type, int image_fps, RSCore::intrinsics intrinsics,
  //     RSCore::video_module_interface::actual_module_config& module_config);

};
}
#endif
