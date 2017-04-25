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

#include "realsense_sp_nodelet.hpp"
#include <chrono>
#include <librealsense/slam/slam_utils.h>


PLUGINLIB_EXPORT_CLASS(realsense_sp::SPNodelet, nodelet::Nodelet)

namespace realsense_sp
{
/*
   * Nodelet constructor.
   */
 
void slam_event_handler::module_output_ready(rs::core::video_module_interface *sender, rs::core::correlated_sample_set *sample)
{
  auto slam = dynamic_cast<rs::slam::slam *>(sender);

  if (!occ_map)
  {
    // This will only happen on the first execution of this callback.
    occ_map = slam->create_occupancy_map(50 * 50);
  }

  // Get the camera pose
  rs::slam::PoseMatrix4f pose;

  slam->get_camera_pose(pose);
  std::string trackingAccuracy;
  rs::slam::tracking_accuracy accuracy = slam->get_tracking_accuracy();
 switch (accuracy)
    {
    case rs::slam::tracking_accuracy::failed:
        trackingAccuracy= "fail";
        break;
    case rs::slam::tracking_accuracy::low:
        trackingAccuracy= "low";
         break;
    case rs::slam::tracking_accuracy::medium: 
        trackingAccuracy= "med";
         break;
    case rs::slam::tracking_accuracy::high:
        trackingAccuracy= "high";
         break;
    default:
        trackingAccuracy= "unknown";
    }

  nav_msgs::Odometry msg;

  msg.header.frame_id = "/odom";
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = camera_frame_id_;
  msg.pose.pose.position.x = pose.m_data[11];
  msg.pose.pose.position.y = -pose.m_data[3];
  msg.pose.pose.position.z = -pose.m_data[7];
  tf::Matrix3x3 mat(pose.m_data[0], pose.m_data[1], pose.m_data[2], pose.m_data[4], pose.m_data[5], pose.m_data[6], pose.m_data[8], pose.m_data[9], pose.m_data[10]);
  tf::Quaternion q;

  mat.getRotation(q);
  msg.pose.pose.orientation.x = q.z();
  msg.pose.pose.orientation.y = -q.x();
  msg.pose.pose.orientation.z = -q.y();
  msg.pose.pose.orientation.w = q.w();

  if (previousTimestamp.toSec() == 0) {
    msg.twist.twist.linear.x = 0;
    msg.twist.twist.linear.y = 0;  
    msg.twist.twist.linear.z = 0;
    msg.twist.twist.angular.x = 0;
    msg.twist.twist.angular.y = 0;
    msg.twist.twist.angular.z = 0;
  } else {
    ros::Duration diff = msg.header.stamp - previousTimestamp;
    double seconds =  diff.toSec();
    tf::Quaternion diffAngle = tf::Quaternion(previousPose.orientation.x,previousPose.orientation.y,previousPose.orientation.z,previousPose.orientation.w) * tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w).inverse();

    msg.twist.twist.linear.x = (previousPose.position.x-msg.pose.pose.position.x)/seconds;
    msg.twist.twist.linear.y = (previousPose.position.y-msg.pose.pose.position.y)/seconds;
    msg.twist.twist.linear.z = (previousPose.position.z-msg.pose.pose.position.z)/seconds;

    tf::Matrix3x3 diffMat(diffAngle);
    diffMat.getRPY(msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z);
    msg.twist.twist.angular.x /=seconds;
    msg.twist.twist.angular.y /=seconds;
    msg.twist.twist.angular.z /=seconds;
  }
  previousTimestamp = msg.header.stamp;
  previousPose.position = msg.pose.pose.position;
  previousPose.orientation = msg.pose.pose.orientation;
  //msg.twist.twist.linear.x = pose.m_data[3];
  //msg.twist.twist.linear.y = pose.m_data[7];     
  //msg.twist.twist.linear.z = pose.m_data[11];

  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = msg.header.stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = msg.pose.pose.position.x;
  odom_trans.transform.translation.y = msg.pose.pose.position.y;
  odom_trans.transform.translation.z = msg.pose.pose.position.z;
  odom_trans.transform.rotation = msg.pose.pose.orientation;
  
  
  br.sendTransform(odom_trans);
  
  

  odomPublisher.publish(msg);

  realsense_sp::Status statusMsg;
  statusMsg.status = trackingAccuracy;
  statusPublisher.publish(statusMsg); 


  // Option to add occupency map, TODO:

  // Get the number of occupancy map tiles that were updated
  /*slam->get_occupancy_map_update(occ_map);
  int count_updated_tiles = occ_map->get_tile_count(); // This will only be > 0 when the tracking accuracy is > low.


  if (count_updated_tiles > 0)
  {
    nav_msgs::OccupancyGrid occMsg;
    // Get the occupany map (with trajectory) as an image and render it in the map window
    unsigned char *map_image;
    unsigned int width, height;
    rs::core::status status = slam->get_occupancy_map_as_rgba(&map_image, &width, &height, true, true);
    const int32_t *data = occ_map->get_tile_coordinates();

    if (status ==rs::core::status::status_no_error)
    {
      cv::Mat image(height, width, CV_8UC4, map_image);
      cv::imwrite("/home/euclid/map.png", image);
      occMsg.header.frame_id = "/map";
      occMsg.info.map_load_time = ros::Time::now();
      occMsg.info.resolution = slam->get_occupancy_map_resolution();
      occMsg.info.width = width;
      occMsg.info.height=height;
      occMsg.info.origin.position = msg.pose.pose.position;
      occMsg.info.origin.orientation =msg.pose.pose.orientation;
      for (int i =5 ; i < count_updated_tiles*3;) {
       // std::cout << "(" << data[i++] << "," << data[i++] << ")=" << data[i++] << std::endl;
      }
      std::cout << "**********************************************" << std::endl;
      std::vector<signed char> a(map_image,map_image+width*height);
      occMsg.data = a;
     
      occPublisher.publish(occMsg); 


    }
    

  }
    cv::waitKey(1);
  */
}
SPNodelet::SPNodelet()
{

  resetClient = nh_.advertiseService("/realsense/slam/reset",&SPNodelet::reset,this);
}

/*
   * Nodelet destructor.
   */
SPNodelet::~SPNodelet()
{
  
}

/*
   * Initialize Nodelet.
   */
 
void SPNodelet::onInit()
{
  ready_ = false;
  actual_config = {};
  memcpy(actual_config.device_info.name, "Intel RealSense ZR300", std::strlen("Intel RealSense ZR300"));
  memcpy(actual_config.device_info.serial, "", std::strlen(""));
  memcpy(actual_config.device_info.firmware, "", std::strlen(""));
  actual_config.device_info.rotation = rs::core::rotation::rotation_0_degree;
  scenePerceptionEventHandler = new slam_event_handler(getPrivateNodeHandle());
  subscribeToCamInfoTopics();
  subscribeToCameraStreams();
}

/*
   * Subscribe to Camera Info Topics.
   */

void SPNodelet::subscribeToCameraStreams()
{
  std::string depthImageStream = "camera/depth/image_raw";
  std::string depthSamplingTopic = "camera/depth/sampling_data";
  std::string fisheyeImageStream = "camera/fisheye/image_raw";
  std::string fisheyeSamplingTopic = "camera/fisheye/sampling_data";
  std::string imuStream = "camera/imu/data_raw";
  
  ROS_INFO_STREAM("Listening on " << depthImageStream);
  ROS_INFO_STREAM("Listening on " << fisheyeImageStream);

  
  
  depth_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(new message_filters::Subscriber<sensor_msgs::Image>(nh_, depthImageStream, 1));
  fisheye_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(new message_filters::Subscriber<sensor_msgs::Image>(nh_, fisheyeImageStream, 1));
  

  depth_sampling_sub_ = std::shared_ptr<message_filters::Subscriber<realsense_camera::SamplingData>>(new message_filters::Subscriber<realsense_camera::SamplingData>(nh_, depthSamplingTopic, 1));
  fisheye_sampling_sub_ = std::shared_ptr<message_filters::Subscriber<realsense_camera::SamplingData>>(new message_filters::Subscriber<realsense_camera::SamplingData>(nh_, fisheyeSamplingTopic, 1));

  depth_sync_ = std::shared_ptr<imageSamplingSyncer>(new imageSamplingSyncer( *depth_sub_, *depth_sampling_sub_,2));
  fisheye_sync_ = std::shared_ptr<imageSamplingSyncer> (new imageSamplingSyncer( *fisheye_sub_, *fisheye_sampling_sub_,2));
  
  

  fisheye_sync_->registerCallback(boost::bind(&SPNodelet::fisheyeMessageCallback, this, _1, _2));
  depth_sync_->registerCallback(boost::bind(&SPNodelet::depthMessageCallback, this, _1, _2));



  ROS_INFO_STREAM("Listening on " << imuStream);
  imu_sub_ = nh_.subscribe(imuStream, 10000, &SPNodelet::imuMessageCallback, this);
}

bool SPNodelet::reset(realsense_sp::Reset::Request & req,realsense_sp::Reset::Response & res) {
    ROS_INFO("Reset!");
    slam_->restart();
    return true;
}
void SPNodelet::depthMessageCallback(const sensor_msgs::ImageConstPtr &depthImageMsg, const realsense_camera::SamplingDataConstPtr& sampling_data )
{

  if (ready_)
  {
    mut_depth.lock();
    rs::core::image_interface *img = rosImageToRSImage(depthImageMsg, rs::core::stream_type::depth,sampling_data);
    rs::core::correlated_sample_set sample_set = {};
    sample_set[rs::core::stream_type::depth] = img;
    if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
    {
      ROS_ERROR("error: failed to process depth sample");
    }

    sample_set[rs::core::stream_type::depth]->release();
    mut_depth.unlock();
  }
}
void SPNodelet::fisheyeMessageCallback(const sensor_msgs::ImageConstPtr &fisheyeImageMsg,const realsense_camera::SamplingDataConstPtr& sampling_data )
{

  if (ready_)
  {
    mut_fisheye.lock();
    rs::core::image_interface *img = rosImageToRSImage(fisheyeImageMsg, rs::core::stream_type::fisheye,sampling_data);
    rs::core::correlated_sample_set sample_set = {};
    sample_set[rs::core::stream_type::fisheye] = img;
    if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
    {
      ROS_ERROR("error: failed to process  fisheye sample");
    }
    sample_set[rs::core::stream_type::fisheye]->release();
    mut_fisheye.unlock();
  }
}
void SPNodelet::imuMessageCallback(const sensor_msgs::ImuConstPtr &imuDataMsg)
{
  if (ready_)
  {
    mut_imu.lock();
    rs::core::motion_type motionType;
    rs::core::correlated_sample_set sample_set = {};
    if (imuDataMsg->linear_acceleration_covariance[0] != -1)
    {
      motionType = rs::core::motion_type::accel;
       sample_set[motionType].frame_number = ++accelCount;
    }
    else
    {
      motionType = rs::core::motion_type::gyro;
      sample_set[motionType].frame_number = ++gyroCount;
    }

    sample_set[motionType].timestamp = (double)(imuDataMsg->header.stamp.nsec * 1e-3);
    sample_set[motionType].type = motionType;
    if (motionType == rs::core::motion_type::accel)
    {
      sample_set[motionType].data[0] = (float)imuDataMsg->linear_acceleration.x;
      sample_set[motionType].data[1] = (float)imuDataMsg->linear_acceleration.y;
      sample_set[motionType].data[2] = (float)imuDataMsg->linear_acceleration.z;
    }
    else if (motionType == rs::core::motion_type::gyro)
    {
      sample_set[motionType].data[0] = (float)imuDataMsg->angular_velocity.x;
      sample_set[motionType].data[1] = (float)imuDataMsg->angular_velocity.y;
      sample_set[motionType].data[2] = (float)imuDataMsg->angular_velocity.z;
    }

    if (slam_->process_sample_set(sample_set) < rs::core::status_no_error)
    {
      ROS_ERROR("error: failed to process imu sample");
    }
    mut_imu.unlock();
  }
}

rs::core::image_interface *SPNodelet::rosImageToRSImage(const sensor_msgs::ImageConstPtr &image, rs::core::stream_type stream, const realsense_camera::SamplingDataConstPtr& sampling_data )
{
  rs::core::image_info img_info;

  img_info.width = image->width;
  img_info.height = image->height;
  img_info.format = (stream == rs::core::stream_type::fisheye ? rs::utils::convert_pixel_format(rs::format::raw8) : rs::utils::convert_pixel_format(rs::format::z16));
  img_info.pitch = image->step;
  cv::Mat img = cv::Mat(img_info.height, img_info.width, stream == rs::core::stream_type::fisheye ? CV_8UC1 : CV_16UC1 , (unsigned char *)image->data.data()).clone();
  return rs::core::image_interface::create_instance_from_raw_data(&img_info,
                                                                  rs::core::image_interface::image_data_with_data_releaser(img.data),
                                                                  stream,
                                                                  rs::core::image_interface::flag::any,
                                                                  (double)(sampling_data->timestamp),
                                                                  sampling_data->frame_number,
                                                                  rs::core::timestamp_domain::microcontroller);
}

void SPNodelet::subscribeToCamInfoTopics()
{

  ROS_INFO_STREAM(nodelet_name_ << "subscribing! ");

  std::string fisheyeCameraInfoStream = "camera/fisheye/camera_info";
  std::string depthCameraInfoStream = "camera/depth/camera_info";
  depth_caminfo_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, depthCameraInfoStream, 1));
  fisheye_caminfo_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, fisheyeCameraInfoStream, 1));

  caminfo_tsync_ = std::shared_ptr<message_filters::Synchronizer<camInfoSyncer>>(new message_filters::Synchronizer<camInfoSyncer>(camInfoSyncer(10), *fisheye_caminfo_sub_, *depth_caminfo_sub_));

  imu_info_client_ = nh_.serviceClient<realsense_camera::GetIMUInfo>("/camera/driver/get_imu_info");
  caminfo_tsync_->registerCallback(boost::bind(&SPNodelet::caminfoCallback, this, _1, _2));

  ROS_INFO_STREAM(nodelet_name_ << "subscribing 2! ");
}

void SPNodelet::initializeIntrinsicData(const sensor_msgs::CameraInfoConstPtr &cameraInfoMsg, rs::core::intrinsics &intrinsic)
{
  intrinsic.width = cameraInfoMsg->width;
  intrinsic.height = cameraInfoMsg->height;
  intrinsic.fx = cameraInfoMsg->K[0];
  intrinsic.fy = cameraInfoMsg->K[4];
  intrinsic.ppx = cameraInfoMsg->K[2];
  intrinsic.ppy = cameraInfoMsg->K[5];

  for (int i = 0; i < 5; i++)
  {
    intrinsic.coeffs[i] = (float)cameraInfoMsg->D[i];
  }
  std::string distortion_model = cameraInfoMsg->distortion_model;
  if (distortion_model.compare("modified_brown_conrady") == 0)
  {
    intrinsic.model = rs::core::distortion_type::modified_brown_conrady;
  }
  else if (distortion_model.compare("none") == 0)
  {
    intrinsic.model = rs::core::distortion_type::none;
  }
  else if (distortion_model.compare("distortion_ftheta") == 0)
  {
    intrinsic.model = rs::core::distortion_type::distortion_ftheta;
  }
  else
  {
    intrinsic.model = rs::core::distortion_type::none;
  }
}
void SPNodelet::initializeExtrinsicData(realsense_camera::GetIMUInfo &extrinsic, rs::core::extrinsics &fe2imu, rs::core::extrinsics &depth2fe)
{
  for (int i = 0; i < 9; ++i)
  {
    fe2imu.rotation[i] = extrinsic.response.fisheye_to_imu_rotation[i];
    depth2fe.rotation[i] = extrinsic.response.depth_to_fisheye_rotation[i];

    if (i < 3)
    {
      fe2imu.translation[i] = extrinsic.response.fisheye_to_imu_translation[i];
      depth2fe.translation[i] = extrinsic.response.depth_to_fisheye_translation[i];
    }
  }

} //end of setExtrinData
void SPNodelet::initializeIMUIntrinsicData(realsense_camera::IMUInfo &imu_res, rs::core::motion_device_intrinsics &motion_intrin)
{
  int index = 0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      motion_intrin.data[i][j] = imu_res.data[index];
      ++index;
    }
    motion_intrin.noise_variances[i] = imu_res.noise_variances[i];
    motion_intrin.bias_variances[i] = imu_res.bias_variances[i];
  }
}
void SPNodelet::getParameters() {
  
    nodelet_name_ = getName();
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
   
    pnh_.param("enable_relocalization", enable_relocalization_, true);
    
  
}
void SPNodelet::initializeSlam()
{

  slam_ = std::unique_ptr<rs::slam::slam>(new rs::slam::slam());
  if(!enable_relocalization_) {
    slam_->stop_relocalization_mapping();
    ROS_INFO("Relocalization - disabled");
  } else {
    slam_->start_relocalization_mapping();
    ROS_INFO("Relocalization - enabled");

  }
  slam_->set_occupancy_map_resolution(0.05);
  slam_->register_event_handler(scenePerceptionEventHandler);
  slam_->register_tracking_event_handler(&trackingEventHandler);

  supported_config = {};
  if (slam_->query_supported_module_config(0, supported_config) < rs::core::status_no_error)
  {
    std::cerr << "error : failed to query the first supported module configuration" << std::endl;
    return;
  }
  ROS_INFO("Done initializing slam nodelet");

}
/*
   * Callback for Camera Info Topics.
   */
void SPNodelet::caminfoCallback(const sensor_msgs::CameraInfoConstPtr &fisheye_caminfo,
                                const sensor_msgs::CameraInfoConstPtr &depth_caminfo)
{
  fisheye_caminfo_sub_->unsubscribe();
  depth_caminfo_sub_->unsubscribe();
  getParameters();
  initializeSlam();

  rs::core::intrinsics depth_intrinsics, fisheye_intrinsics;
  initializeIntrinsicData(fisheye_caminfo, fisheye_intrinsics);
  initializeIntrinsicData(depth_caminfo, depth_intrinsics);
  /*
     * Get imu intrinsics
     */
  realsense_camera::GetIMUInfo imu_info_srv;
  if (imu_info_client_.call(imu_info_srv))
  {
    NODELET_INFO("imu info got");
    rs::core::motion_device_intrinsics acc, gyro;

    initializeIMUIntrinsicData(imu_info_srv.response.accel, acc);
    initializeIMUIntrinsicData(imu_info_srv.response.gyro, gyro);
    std::map<rs::core::stream_type, rs::core::intrinsics> intrinsics;
    intrinsics[rs::core::stream_type::depth] = depth_intrinsics;
    intrinsics[rs::core::stream_type::fisheye] = fisheye_intrinsics;

    std::map<rs::core::motion_type, rs::core::motion_device_intrinsics> motion_intrinsics;
    motion_intrinsics[rs::core::motion_type::accel] = acc;
    motion_intrinsics[rs::core::motion_type::gyro] = gyro;
    rs::core::extrinsics fe2motion, dep2fe;
    for (auto &motion : {rs::core::motion_type::gyro, rs::core::motion_type::accel})
    {

      actual_config[motion].is_enabled = true;
      actual_config[motion].intrinsics = motion_intrinsics[motion];
    }
    initializeExtrinsicData(imu_info_srv, fe2motion, dep2fe);

    for (auto &stream : {rs::core::stream_type::depth, rs::core::stream_type::fisheye})
    {
      auto &supported_stream_config = supported_config[stream];
      rs::core::video_module_interface::actual_image_stream_config &actual_stream_config = actual_config[stream];
      actual_stream_config.size.width = intrinsics[stream].width;
      actual_stream_config.size.height = intrinsics[stream].height;
      actual_stream_config.frame_rate = supported_stream_config.frame_rate;
      actual_stream_config.intrinsics = intrinsics[stream];
      actual_stream_config.is_enabled = true;
    }
    actual_config[rs::core::motion_type::accel].sample_rate = 125;
    actual_config[rs::core::motion_type::gyro].sample_rate = 200;
    actual_config[rs::core::stream_type::fisheye].extrinsics_motion = fe2motion;
    actual_config[rs::core::stream_type::fisheye].extrinsics = dep2fe;
    int code;
    if ((code = slam_->set_module_config(actual_config)) < rs::core::status_no_error)
    {
      NODELET_ERROR("error : failed to set the enabled module configuration: ");
      std::cout << "code: " << code << std::endl;
      return;
    }
    ready_ = true;

  }
  else
  {
    NODELET_ERROR("imu info missed");
  }
}

void SPNodelet::printConfig(rs::core::video_module_interface::actual_module_config &slam_config)
{
  for (auto &stream : {rs::core::stream_type::fisheye, rs::core::stream_type::depth})
  {
    std::cout << "width: " << actual_config[stream].size.width << std::endl;
    std::cout << "height: " << actual_config[stream].size.height << std::endl;
    std::cout << "fps: " << actual_config[stream].frame_rate << std::endl;
    std::cout << "is_enabled: " << actual_config[stream].is_enabled << std::endl;

    std::cout << "width: " << actual_config[stream].intrinsics.width << std::endl;
    std::cout << "height: " << actual_config[stream].intrinsics.height << std::endl;

    std::cout << "fx: " << actual_config[stream].intrinsics.fx << std::endl;
    std::cout << "fy: " << actual_config[stream].intrinsics.fy << std::endl;
    std::cout << "ppx: " << actual_config[stream].intrinsics.ppx << std::endl;
    std::cout << "ppy: " << actual_config[stream].intrinsics.ppy << std::endl;
    for (int i = 0; i < 5; i++)
    {

      std::cout << "coeffs[" << i << "]=" << actual_config[stream].intrinsics.coeffs[i] << std::endl;
    }
  }
  std::cout << "======= Motion:" << std::endl;
  std::cout << "accel_sample_rate: " << actual_config[rs::core::motion_type::accel].sample_rate << std::endl;
  std::cout << "gyro_sample_rate: " << actual_config[rs::core::motion_type::gyro].sample_rate << std::endl;
  std::cout << "Extrinsic - fisheye to motion" << std::endl;
  for (int i = 0; i < 9; ++i)
  {
    std::cout << "Rotation: " << actual_config[rs::core::stream_type::fisheye].extrinsics_motion.rotation[i] << std::endl;
  }
  for (int i = 0; i < 3; ++i)
  {
    std::cout << "translation: " << actual_config[rs::core::stream_type::fisheye].extrinsics_motion.translation[i] << std::endl;
  }
  std::cout << "Extrinsic - fisheye to depth" << std::endl;
  for (int i = 0; i < 9; ++i)
  {
    std::cout << "Rotation: " << actual_config[rs::core::stream_type::fisheye].extrinsics.rotation[i] << std::endl;
  }
  for (int i = 0; i < 3; ++i)
  {
    std::cout << "translation: " << actual_config[rs::core::stream_type::fisheye].extrinsics.translation[i] << std::endl;
  }

  std::cout << "Motion intrinsic" << std::endl;

  for (auto &motion : {rs::core::motion_type::gyro, rs::core::motion_type::accel})
  {
    std::cout << "data" << std::endl;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        std::cout << "Intrinsic: " << actual_config[motion].intrinsics.data[i][j] << std::endl;
      }
    }
    for (int i = 0; i < 3; ++i)
    {

      std::cout << "noise: " << actual_config[motion].intrinsics.noise_variances[i] << std::endl;
    }
    for (int i = 0; i < 3; ++i)
    {

      std::cout << "bias: " << actual_config[motion].intrinsics.bias_variances[i] << std::endl;
    }
  }
}
}




