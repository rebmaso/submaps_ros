/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Apr 27, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Publisher.cpp
 * @brief Source file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include "Publisher.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/package.h>
#pragma GCC diagnostic pop
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include <okvis/FrameTypedefs.hpp>

Eigen::Vector3d target_pos(0,0,0);
Eigen::Vector3d target_vel(0,0,0);

/// \brief okvis Main namespace of this package.
// namespace okvis {

// Default constructor.
Publisher::Publisher()
    : nh_(nullptr),
      ctr2_(0)
{
}

Publisher::~Publisher()
{
  // close file
  if (csvLandmarksFile_) {
    // write down also the current landmarks
    if (csvLandmarksFile_->good()) {
      for (size_t l = 0; l < pointsMatched2_.size(); ++l) {
        Eigen::Vector4d landmark = pointsMatched2_.at(l).point;
        *csvLandmarksFile_ << std::setprecision(19) << pointsMatched2_.at(l).id
            << ", " << std::scientific << std::setprecision(18) << landmark[0]
            << ", " << landmark[1] << ", " << landmark[2] << ", " << landmark[3]
            << ", " << pointsMatched2_.at(l).quality << std::endl;
      }
    }
    csvLandmarksFile_->close();
  }
  if (csvFile_)
    csvFile_->close();
}

// Constructor. Calls setNodeHandle().
Publisher::Publisher(ros::NodeHandle& nh)
    : Publisher()
{ 
  setNodeHandle(nh);
}

// Set the node handle and advertise topics.
void Publisher::setNodeHandle(ros::NodeHandle& nh)
{
  nh_ = &nh;

  // advertise
  pubPointsMatched_ = nh_->advertise<sensor_msgs::PointCloud2>(
      "okvis_points_matched", 1);
  pubPointsUnmatched_ = nh_->advertise<sensor_msgs::PointCloud2>(
      "okvis_points_unmatched", 1);
  pubPointsTransferred_ = nh_->advertise<sensor_msgs::PointCloud2>(
      "okvis_points_transferred", 1);
  pubObometry_ = nh_->advertise<nav_msgs::Odometry>("okvis_odometry", 1);
  pubPath_ = nh_->advertise<nav_msgs::Path>("okvis_path", 1);
  pubTransform_ = nh_->advertise<geometry_msgs::TransformStamped>(
      "okvis_transform", 1);
  pubMarkerTransform_ = nh_->advertise<geometry_msgs::TransformStamped>(
      "okvis_marker_transform", 1);
  pubMarkerOdometry_ = nh_->advertise<nav_msgs::Odometry>(
      "okvis_marker_odometry", 1);
  pubMesh_ = nh_->advertise<visualization_msgs::Marker>( "okvis_mesh", 0 );
  // where to get the mesh from
  std::string mesh_file;
  if (nh_->getParam("mesh_file", mesh_file)) {
    meshMsg_.mesh_resource = "package://okvis_ros/meshes/"+mesh_file;
  } else {
    LOG(INFO) << "no mesh found for visualisation, set ros param mesh_file, if desired";
    meshMsg_.mesh_resource = "";
  }
  
  // stuff by Tommaso
  pubKeyframes_ = nh_->advertise<visualization_msgs::MarkerArray>("okvis_keyframe_poses", 0);
  pubSubmaps_ = nh_->advertise<visualization_msgs::MarkerArray>( "se_submaps", 0 );
  pubOMPLPath_ = nh_->advertise<nav_msgs::Path>( "ompl_path", 0 );
  map_free_pub_ = nh_->advertise<visualization_msgs::Marker>("se_map_free", 1);
  map_occupied_pub_ = nh_->advertise<visualization_msgs::Marker>("se_map_occupied", 1);
  map_unknown_pub_ = nh_->advertise<visualization_msgs::Marker>("se_map_unknown", 1);

}

// Write CSV header.
bool Publisher::writeCsvDescription()
{
  if (!csvFile_)
    return false;
  if (!csvFile_->good())
    return false;
  *csvFile_ << "timestamp" << ", " << "p_WS_W_x" << ", " << "p_WS_W_y" << ", "
            << "p_WS_W_z" << ", " << "q_WS_x" << ", " << "q_WS_y" << ", "
            << "q_WS_z" << ", " << "q_WS_w" << ", " << "v_WS_W_x" << ", "
            << "v_WS_W_y" << ", " << "v_WS_W_z" << ", " << "b_g_x" << ", "
            << "b_g_y" << ", " << "b_g_z" << ", " << "b_a_x" << ", " << "b_a_y"
            << ", " << "b_a_z" << std::endl;
  return true;
}

// Write CSV header for landmarks file.
bool Publisher::writeLandmarksCsvDescription()
{
  if (!csvLandmarksFile_)
    return false;
  if (!csvLandmarksFile_->good())
    return false;
  *csvLandmarksFile_ << ", " << "id" << ", " << "l_x" << ", " << "l_y" << ", "
                     << "l_z" << ", " << "l_w" << ", " << "quality, "
                     << "distance" << std::endl;
  return true;
}

// Set an odometry output CSV file.
bool Publisher::setCsvFile(std::fstream& csvFile)
{
  if (csvFile_) {
    csvFile_->close();
  }
  csvFile_.reset(&csvFile);
  writeCsvDescription();
  return csvFile_->good();
}
// Set an odometry output CSV file.
bool Publisher::setCsvFile(std::string& csvFileName)
{
  csvFile_.reset(new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeCsvDescription();
  return csvFile_->good();
}
// Set an odometry output CSV file.
bool Publisher::setCsvFile(std::string csvFileName)
{
  csvFile_.reset(new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeCsvDescription();
  return csvFile_->good();
}

// Set a CVS file where the landmarks will be saved to.
bool Publisher::setLandmarksCsvFile(std::fstream& csvFile)
{
  if (csvLandmarksFile_) {
    csvLandmarksFile_->close();
  }
  csvLandmarksFile_.reset(&csvFile);
  writeLandmarksCsvDescription();
  return csvLandmarksFile_->good();
}
// Set a CVS file where the landmarks will be saved to.
bool Publisher::setLandmarksCsvFile(std::string& csvFileName)
{
  csvLandmarksFile_.reset(
      new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeLandmarksCsvDescription();
  return csvLandmarksFile_->good();
}
// Set a CVS file where the landmarks will be saved to.
bool Publisher::setLandmarksCsvFile(std::string csvFileName)
{
  csvLandmarksFile_.reset(
      new std::fstream(csvFileName.c_str(), std::ios_base::out));
  writeLandmarksCsvDescription();
  return csvLandmarksFile_->good();
}

// Set the pose message that is published next.
void Publisher::setPose(const okvis::kinematics::Transformation& T_WS)
{

  okvis::kinematics::Transformation T;
  /// \todo get these kinds of parameters from ros params
  //if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    poseMsg_.child_frame_id = "okvis_tracking";
    //T = parameters_.publishing.T_Wc_W * T_WS;
    T = T_WS;
  /*} else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    poseMsg_.child_frame_id = "body";
    T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();
  } else {
    LOG(ERROR) <<
        "Pose frame does not exist for publishing. Choose 'S' or 'B'.";
    poseMsg_.child_frame_id = "body";
    T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();
  }*/

  poseMsg_.header.frame_id = "world";
  poseMsg_.header.stamp = _t;
  if ((ros::Time::now() - _t).toSec() > 10.0)
    poseMsg_.header.stamp = ros::Time::now();

  // fill orientation
  Eigen::Quaterniond q = T.q();
  poseMsg_.transform.rotation.x = q.x();
  poseMsg_.transform.rotation.y = q.y();
  poseMsg_.transform.rotation.z = q.z();
  poseMsg_.transform.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = T.r();
  poseMsg_.transform.translation.x = r[0];
  poseMsg_.transform.translation.y = r[1];
  poseMsg_.transform.translation.z = r[2];

  // also do the mesh
  /*if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    meshMsg_.child_frame_id = "sensor";
  } else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    meshMsg_.child_frame_id = "body";
  } else {
    meshMsg_.child_frame_id = "body";
  }*/
  meshMsg_.header.frame_id = "body";
  meshMsg_.header.stamp = _t;
  meshMsg_.type = visualization_msgs::Marker::MESH_RESOURCE;
  if ((ros::Time::now() - _t).toSec() > 10.0)
    meshMsg_.header.stamp = ros::Time::now();

  // fill orientation
  meshMsg_.pose.orientation.x = 0;
  meshMsg_.pose.orientation.y = 0;
  meshMsg_.pose.orientation.z = 0;
  meshMsg_.pose.orientation.w = 1;

  // fill position
  meshMsg_.pose.position.x = 0;
  meshMsg_.pose.position.y = 0;
  meshMsg_.pose.position.z = 0;

  // scale -- needed
  meshMsg_.scale.x = 1.0;
  meshMsg_.scale.y = 1.0;
  meshMsg_.scale.z = 1.0;
	
	meshMsg_.action = visualization_msgs::Marker::ADD;
	meshMsg_.color.a = 1.0; // Don't forget to set the alpha!
	meshMsg_.color.r = 1.0;
	meshMsg_.color.g = 1.0;
	meshMsg_.color.b = 1.0;

  // embedded material / colour
  //meshMsg_.mesh_use_embedded_materials = true;
}

// Set the odometry message that is published next.
void Publisher::setOdometry(const okvis::kinematics::Transformation& T_WS,
                            const okvis::SpeedAndBiases& speedAndBiases,
                            const Eigen::Vector3d& omega_S)
{

  // header.frame_id is the frame in which the pose is given. I.e. world frame in our case
  // child_frame_id is the frame in which the twist part of the odometry message is given.
  // see also nav_msgs/Odometry Message documentation

// std::cout << speedAndBiases.tail<3>().transpose() << std::endl;

  odometryMsg_.header.stamp = _t;
  okvis::kinematics::Transformation T; // the pose to be published. T_WS or T_WB depending on 'trackedBodyFrame'
  //Eigen::Vector3d omega_W = parameters_.publishing.T_Wc_W.C() * T_WS.C() * omega_S; 
  /// \todo properly form ros params
  Eigen::Vector3d omega_W = T_WS.C() * omega_S;
  Eigen::Vector3d t_W_ofFrame;  // lever arm in W-system
  Eigen::Vector3d v_W_ofFrame;  // velocity in W-system. v_S_in_W or v_B_in_W

  //if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    odometryMsg_.header.frame_id = "world";
    //T = parameters_.publishing.T_Wc_W * T_WS;
    T = T_WS;
    t_W_ofFrame.setZero(); // r_SS_in_W
    //v_W_ofFrame = parameters_.publishing.T_Wc_W.C()*speedAndBiases.head<3>();  // world-centric speedAndBiases.head<3>()
    v_W_ofFrame = speedAndBiases.head<3>();  // world-centric speedAndBiases.head<3>()
  /*} else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    odometryMsg_.header.frame_id = "world";
    T = parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse();
    t_W_ofFrame = (parameters_.publishing.T_Wc_W * T_WS * parameters_.imu.T_BS.inverse()).r() - (parameters_.publishing.T_Wc_W * T_WS).r();  // r_BS_in_W
    v_W_ofFrame = parameters_.publishing.T_Wc_W.C()*speedAndBiases.head<3>() + omega_W.cross(t_W_ofFrame);  // world-centric speedAndBiases.head<3>()
  } else {
    LOG(ERROR) <<
        "Pose frame does not exist for publishing. Choose 'S' or 'B'.";
    odometryMsg_.header.frame_id = "world";
    T = parameters_.publishing.T_Wc_W * T_WS;
    t_W_ofFrame.setZero(); // r_SS_in_W
    v_W_ofFrame = parameters_.publishing.T_Wc_W.C()*speedAndBiases.head<3>();  // world-centric speedAndBiases.head<3>()
  }*/

  // fill orientation
  Eigen::Quaterniond q = T.q();
  odometryMsg_.pose.pose.orientation.x = q.x();
  odometryMsg_.pose.pose.orientation.y = q.y();
  odometryMsg_.pose.pose.orientation.z = q.z();
  odometryMsg_.pose.pose.orientation.w = q.w();

  // fill position
  Eigen::Vector3d r = T.r();
  odometryMsg_.pose.pose.position.x = r[0];
  odometryMsg_.pose.pose.position.y = r[1];
  odometryMsg_.pose.pose.position.z = r[2];

  Eigen::Matrix3d C_v;
  Eigen::Matrix3d C_omega;
  /// \todo properly from ros params
  //if (parameters_.publishing.velocitiesFrame == FrameName::S) {
    //C_v = (parameters_.publishing.T_Wc_W * T_WS).inverse().C();
    C_v = (T_WS).inverse().C();
    C_omega.setIdentity();
    odometryMsg_.child_frame_id = "okvis_tracking";
  /*} else if (parameters_.publishing.velocitiesFrame == FrameName::B) {
    C_v = (parameters_.imu.T_BS * T_WS.inverse()).C() * parameters_.publishing.T_Wc_W.inverse().C();
    C_omega = parameters_.imu.T_BS.C();
    odometryMsg_.child_frame_id = "body";
  } else if (parameters_.publishing.velocitiesFrame == FrameName::Wc) {
    C_v.setIdentity();
    C_omega = parameters_.publishing.T_Wc_W.C() * T_WS.C();
    odometryMsg_.child_frame_id = "world";
  } else {
    LOG(ERROR) <<
        "Speeds frame does not exist for publishing. Choose 'S', 'B', or 'Wc'.";
    C_v = (parameters_.imu.T_BS * T_WS.inverse()).C() * parameters_.publishing.T_Wc_W.inverse().C();
    C_omega = parameters_.imu.T_BS.C();
    odometryMsg_.child_frame_id = "body";
  }*/

  // fill velocity
  Eigen::Vector3d v = C_v * v_W_ofFrame;    // v_S_in_'speedsInThisFrame' or v_B_in_'speedsInThisFrame'
  odometryMsg_.twist.twist.linear.x = v[0];
  odometryMsg_.twist.twist.linear.y = v[1];
  odometryMsg_.twist.twist.linear.z = v[2];

  // fill angular velocity
  Eigen::Vector3d omega = C_omega * omega_S;  // omega_in_'speedsInThisFrame'
  odometryMsg_.twist.twist.angular.x = omega[0];
  odometryMsg_.twist.twist.angular.y = omega[1];
  odometryMsg_.twist.twist.angular.z = omega[2];

  // linear acceleration ?? - would also need point of percussion mapping!!
}


// Set the points that are published next.
void Publisher::setPoints(const okvis::MapPointVector& pointsMatched,
                          const okvis::MapPointVector& pointsUnmatched,
                          const okvis::MapPointVector& pointsTransferred)
{
  pointsMatched2_.clear();
  pointsMatched2_ = pointsMatched;
  pointsMatched_.clear();
  pointsUnmatched_.clear();
  pointsTransferred_.clear();

  // transform points into custom world frame:
  const Eigen::Matrix4d T_Wc_W; // = parameters_.publishing.T_Wc_W.T();
  /// \todo properly from ros params -- also landmark thresholds below

  for (size_t i = 0; i < pointsMatched.size(); ++i) {
    // check infinity
    if (fabs((double) (pointsMatched[i].point[3])) < 1.0e-8)
      continue;

    // check quality
    //if (pointsMatched[i].quality < parameters_.publishing.landmarkQualityThreshold)
    if (pointsMatched[i].quality < 0.01)
      continue;

    pointsMatched_.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = T_Wc_W * pointsMatched[i].point;
    pointsMatched_.back().x = point[0] / point[3];
    pointsMatched_.back().y = point[1] / point[3];
    pointsMatched_.back().z = point[2] / point[3];
    //pointsMatched_.back().g = 255
    //    * (std::min(parameters_.publishing.maxLandmarkQuality, (float)pointsMatched[i].quality)
    //        / parameters_.publishing.maxLandmarkQuality);
    pointsMatched_.back().g = 255
        * (std::min(0.1f, (float)pointsMatched[i].quality)
            / 0.1f);
  }
  pointsMatched_.header.frame_id = "world";

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  std_msgs::Header header;
  header.stamp = _t;
  pointsMatched_.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  pointsMatched_.header.stamp=_t;
#endif

  for (size_t i = 0; i < pointsUnmatched.size(); ++i) {
    // check infinity
    if (fabs((double) (pointsUnmatched[i].point[3])) < 1.0e-8)
      continue;

    // check quality
    //if (pointsUnmatched[i].quality < parameters_.publishing.landmarkQualityThreshold)
    if (pointsUnmatched[i].quality < 0.01)
      continue;

    pointsUnmatched_.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = T_Wc_W * pointsUnmatched[i].point;
    pointsUnmatched_.back().x = point[0] / point[3];
    pointsUnmatched_.back().y = point[1] / point[3];
    pointsUnmatched_.back().z = point[2] / point[3];
    //pointsUnmatched_.back().b = 255
    //    * (std::min(parameters_.publishing.maxLandmarkQuality, (float)pointsUnmatched[i].quality)
    //        / parameters_.publishing.maxLandmarkQuality);
    pointsUnmatched_.back().b = 255
        * (std::min(0.1f, (float)pointsUnmatched[i].quality)
            / 0.1f);
  }
  pointsUnmatched_.header.frame_id = "world";

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  pointsUnmatched_.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  pointsUnmatched_.header.stamp=_t;
#endif

  for (size_t i = 0; i < pointsTransferred.size(); ++i) {
    // check infinity

    if (fabs((double) (pointsTransferred[i].point[3])) < 1.0e-10)
      continue;

    // check quality
    //if (pointsTransferred[i].quality < parameters_.publishing.landmarkQualityThreshold)
    if (pointsTransferred[i].quality < 0.01)
      continue;

    pointsTransferred_.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = T_Wc_W * pointsTransferred[i].point;
    pointsTransferred_.back().x = point[0] / point[3];
    pointsTransferred_.back().y = point[1] / point[3];
    pointsTransferred_.back().z = point[2] / point[3];
    //float intensity = std::min(parameters_.publishing.maxLandmarkQuality,
    //                           (float)pointsTransferred[i].quality)
    //    / parameters_.publishing.maxLandmarkQuality;
    float intensity = std::min(0.1f,
                               (float)pointsTransferred[i].quality)
        / 0.1f;
    pointsTransferred_.back().r = 255 * intensity;
    pointsTransferred_.back().g = 255 * intensity;
    pointsTransferred_.back().b = 255 * intensity;

    //_omfile << point[0] << " " << point[1] << " " << point[2] << ";" <<std::endl;

  }
  pointsTransferred_.header.frame_id = "world";
  pointsTransferred_.header.seq = ctr2_++;

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  pointsTransferred_.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  pointsTransferred_.header.stamp=_t;
#endif
}

// Publish the pose.
void Publisher::publishPose()
{
  //if ((_t - lastOdometryTime2_).toSec() < 1.0 / parameters_.publishing.publishRate)
  //  return;  // control the publish rate
  /// \todo get this from ros params
  pubTf_.sendTransform(poseMsg_);
  if(!meshMsg_.mesh_resource.empty())
    pubMesh_.publish(meshMsg_);  //publish stamped mesh
  lastOdometryTime2_ = _t;  // remember
}

// Publish the last set odometry.
void Publisher::publishOdometry()
{
  //if ((_t - lastOdometryTime_).toSec() < 1.0 / parameters_.publishing.publishRate)
  //  return;  // control the publish rate
  /// \todo get this from ros params
  pubObometry_.publish(odometryMsg_);
  if(!meshMsg_.mesh_resource.empty())
    pubMesh_.publish(meshMsg_);  //publish stamped mesh
  lastOdometryTime_ = _t;  // remember
}

// Publish the T_WS transform.
void Publisher::publishTransform()
{
  //if ((_t - lastTransfromTime_).toSec() < 1.0 / parameters_.publishing.publishRate)
  //  return;  // control the publish rate
  /// \todo get this from ros params
  pubTransform_.publish(poseMsg_);  //publish stamped transform for MSF
  lastTransfromTime_ = _t;  // remember
}

// Set and publish pose.
void Publisher::publishStateAsCallback(
    const okvis::Time & t, const okvis::kinematics::Transformation & T_WS)
{
  setTime(t);
  setPose(T_WS);  // TODO: provide setters for this hack
  publishPose();
}
// Set and publish full state.
void Publisher::publishFullStateAsCallback(
    const okvis::Time & t, const okvis::kinematics::Transformation & T_WS,
    const Eigen::Matrix<double, 9, 1> & speedAndBiases,
    const Eigen::Matrix<double, 3, 1> & omega_S)
{
  setTime(t);
  setOdometry(T_WS, speedAndBiases, omega_S);  // TODO: provide setters for this hack
  setPath(T_WS);
  publishOdometry();
  publishTransform();
  publishPath();
}

void Publisher::processState(const State& state, const TrackingState & trackingState) 
{
  Eigen::Matrix<double, 9, 1> speedAndBiases;
  speedAndBiases.head<3>() = state.v_W;
  speedAndBiases.segment<3>(3) = state.b_g;
  speedAndBiases.tail<3>() = state.b_a;

  setTime(state.timestamp);
  setOdometry(state.T_WS, speedAndBiases, state.omega_S);  // TODO: provide setters for this hack
  setPath(state.T_WS);
  setPose(state.T_WS);
  publishOdometry();
  publishTransform();
  publishPath();
  publishPose();
}

// Set and write full state to CSV file.
void Publisher::csvSaveFullStateAsCallback(
    const okvis::Time & t, const okvis::kinematics::Transformation & T_WS,
    const Eigen::Matrix<double, 9, 1> & speedAndBiases,
    const Eigen::Matrix<double, 3, 1> & omega_S)
{
  setTime(t);
  setOdometry(T_WS, speedAndBiases, omega_S);  // TODO: provide setters for this hack
  if (csvFile_) {
    //LOG(INFO)<<"filePtr: ok; ";
    if (csvFile_->good()) {
      //LOG(INFO)<<"file: good.";
      Eigen::Vector3d p_WS_W = T_WS.r();
      Eigen::Quaterniond q_WS = T_WS.q();
      std::stringstream time;
      time << t.sec << std::setw(9) << std::setfill('0') << t.nsec;
      *csvFile_ << time.str() << ", " << std::scientific
          << std::setprecision(18) << p_WS_W[0] << ", " << p_WS_W[1] << ", "
          << p_WS_W[2] << ", " << q_WS.x() << ", " << q_WS.y() << ", "
          << q_WS.z() << ", " << q_WS.w() << ", " << speedAndBiases[0] << ", "
          << speedAndBiases[1] << ", " << speedAndBiases[2] << ", "
          << speedAndBiases[3] << ", " << speedAndBiases[4] << ", "
          << speedAndBiases[5] << ", " << speedAndBiases[6] << ", "
          << speedAndBiases[7] << ", " << speedAndBiases[8] << std::endl;
    }
  }
}

// Set and write full state including camera extrinsics to file.
void Publisher::csvSaveFullStateWithExtrinsicsAsCallback(
    const okvis::Time & t,
    const okvis::kinematics::Transformation & T_WS,
    const Eigen::Matrix<double, 9, 1> & speedAndBiases,
    const Eigen::Matrix<double, 3, 1> & omega_S,
    const std::vector<okvis::kinematics::Transformation,
        Eigen::aligned_allocator<okvis::kinematics::Transformation> > & extrinsics)
{
  setTime(t);
  setOdometry(T_WS, speedAndBiases, omega_S);  // TODO: provide setters for this hack
  if (csvFile_) {
    if (csvFile_->good()) {
      Eigen::Vector3d p_WS_W = T_WS.r();
      Eigen::Quaterniond q_WS = T_WS.q();
      std::stringstream time;
      time << t.sec << std::setw(9) << std::setfill('0') << t.nsec;
      *csvFile_ << time.str() << ", " << std::scientific
          << std::setprecision(18) << p_WS_W[0] << ", " << p_WS_W[1] << ", "
          << p_WS_W[2] << ", " << q_WS.x() << ", " << q_WS.y() << ", "
          << q_WS.z() << ", " << q_WS.w() << ", " << speedAndBiases[0] << ", "
          << speedAndBiases[1] << ", " << speedAndBiases[2] << ", "
          << speedAndBiases[3] << ", " << speedAndBiases[4] << ", "
          << speedAndBiases[5] << ", " << speedAndBiases[6] << ", "
          << speedAndBiases[7] << ", " << speedAndBiases[8];
      for (size_t i = 0; i < extrinsics.size(); ++i) {
        Eigen::Vector3d p_SCi = extrinsics[i].r();
        Eigen::Quaterniond q_SCi = extrinsics[i].q();
        *csvFile_ << ", " << p_SCi[0] << ", " << p_SCi[1] << ", " << p_SCi[2]
            << ", " << q_SCi.x() << ", " << q_SCi.y() << ", " << q_SCi.z()
            << ", " << q_SCi.w();
      }
      *csvFile_ << std::endl;
    }
  }
}

// Set and publish landmarks.
void Publisher::publishLandmarksAsCallback(
    const okvis::Time & /*t*/, const okvis::MapPointVector & actualLandmarks,
    const okvis::MapPointVector & transferredLandmarks)
{
  //if(parameters_.publishing.publishLandmarks){
    okvis::MapPointVector empty;
    setPoints(actualLandmarks, empty, transferredLandmarks);
    publishPoints();
  //}
}


void Publisher::publishKeyframesAsCallback(const State &latestState, const TrackingState &latestTrackingState,const StateVector &keyframeStates) {

  visualization_msgs::MarkerArray KFarraymsg_;

  for (int i = 0; i < keyframeStates.size(); i++) // iterate over keyframes stdvector
  {
  
  // single marker
  visualization_msgs::Marker KFmsg_;

  // header
  KFmsg_.header.frame_id = "world";
  // time conversion from okvis
  ros::Time KFtimestamp = ros::Time(keyframeStates[i].timestamp.sec, keyframeStates[i].timestamp.nsec);
  KFmsg_.header.stamp = KFtimestamp;
  if ((ros::Time::now() - KFtimestamp).toSec() > 10.0)
    KFmsg_.header.stamp = ros::Time::now(); // weird hack
  KFmsg_.ns = "kf_ns";
  KFmsg_.id = keyframeStates[i].id.value(); // id of the keyframe
  KFmsg_.type = visualization_msgs::Marker::ARROW;
  // KFmsg_.type = visualization_msgs::Marker::SPHERE;
  KFmsg_.action = visualization_msgs::Marker::ADD;
  // orientation
  Eigen::Quaterniond q = keyframeStates[i].T_WS.q();
  KFmsg_.pose.orientation.x = q.x();
  KFmsg_.pose.orientation.y = q.y();
  KFmsg_.pose.orientation.z = q.z();
  KFmsg_.pose.orientation.w = q.w();
  // position
  Eigen::Vector3d r = keyframeStates[i].T_WS.r();
  KFmsg_.pose.position.x = r[0];
  KFmsg_.pose.position.y = r[1];
  KFmsg_.pose.position.z = r[2];
  // scale
  KFmsg_.scale.x = 0.25;
  KFmsg_.scale.y = 0.02;
  KFmsg_.scale.z = 0.02;
  KFmsg_.color.a = 1.0;
  KFmsg_.color.r = 1.0;
  KFmsg_.color.g = 0.0;
  KFmsg_.color.b = 0.0;
  KFmsg_.lifetime = ros::Duration(0.5); // lasts for 0.5 sec
  KFmsg_.frame_locked = true;

  // push current KF pose to the array
  KFarraymsg_.markers.push_back(KFmsg_);

  }

  // publish the whole array, once it has been filled
  pubKeyframes_.publish(KFarraymsg_);
  
}

void Publisher::setMeshesPath(std::string meshesDir){
 meshesDir_ = meshesDir;
}

void Publisher::publishSubmapMeshesAsCallback(std::unordered_map<uint64_t, Transformation> submapPoseLookup) 
{
  
  visualization_msgs::MarkerArray submaparraymsg_; // put all submaps here

  // get array of all ply files:
  
  std::string extension = ".ply";
  boost::filesystem::path root(meshesDir_);
  std::vector<boost::filesystem::path> paths_ply; // paths of all ply files in directory
  

  if (boost::filesystem::exists(root) && boost::filesystem::is_directory(root))
  {
      for (auto const & entry : boost::filesystem::recursive_directory_iterator(root))
      {
          if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == extension)
              paths_ply.emplace_back(entry.path());
      }
  }

  // iterate over plys and convert them to stl. then delete

  for (int i = 0; i < paths_ply.size(); i++)
  {  
      pcl::PolygonMesh mesh; 
      std::filesystem::path output_file(paths_ply[i].string()); // we save the stl mesh here
      output_file.replace_extension(".stl");

      // load mesh
      // std::cout << "(publisher) loading" << paths[i].string() << " as mesh \n \n \n";
      if (!pcl::io::loadPolygonFile(paths_ply[i].string(), mesh)) std::cout << "(publisher) error loading ply mesh! \n";
      // save mesh as .stl
      // std::cout << "(publisher) saving mesh as " << output_file.string() << " \n \n \n"; // exporting in binary format
      if (!pcl::io::savePolygonFile(output_file.string(), mesh, true)) std::cout << "(publisher) error saving stl file! \n";
      
      // delete current ply
      boost::filesystem::remove(paths_ply[i]);
  }

  // get array of all stl files:

  extension = ".stl";
  std::vector<boost::filesystem::path> paths_stl; // paths of all stl files in directory
  

  if (boost::filesystem::exists(root) && boost::filesystem::is_directory(root))
  {
      for (auto const & entry : boost::filesystem::recursive_directory_iterator(root))
      {
          if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == extension)
              paths_stl.emplace_back(entry.path());
      }
  }

  // iterate over stls and load  single submap msg:

  for (int i = 0; i < paths_stl.size(); i++)
  {  
      visualization_msgs::Marker submapmsg_;
      // load marker msg:
      // header
      submapmsg_.header.frame_id = "world";
      submapmsg_.header.stamp = ros::Time::now();
      submapmsg_.ns = "submap_ns";
      submapmsg_.id = stoi(paths_stl[i].stem().string()); // id of the keyframe (filename wo extension)
      submapmsg_.type = visualization_msgs::Marker::MESH_RESOURCE;
      submapmsg_.mesh_resource = "file://" + paths_stl[i].string(); // resource_retriever url syntax
      submapmsg_.action = visualization_msgs::Marker::ADD;
      submapmsg_.lifetime = ros::Duration(0.0); // lasts forever (gets updated though)
      submapmsg_.frame_locked = true;
      // orientation
      Eigen::Quaterniond q = submapPoseLookup[std::stoull(paths_stl[i].stem().string())].q(); // stoull to convert string -> uint64_t
      submapmsg_.pose.orientation.x = q.x();
      submapmsg_.pose.orientation.y = q.y();
      submapmsg_.pose.orientation.z = q.z();
      submapmsg_.pose.orientation.w = q.w();
      // position
      Eigen::Vector3d r = submapPoseLookup[std::stoull(paths_stl[i].stem().string())].r();
      submapmsg_.pose.position.x = r[0];
      submapmsg_.pose.position.y = r[1];
      submapmsg_.pose.position.z = r[2];
      // scale
      submapmsg_.scale.x = 1.0;
      submapmsg_.scale.y = 1.0;
      submapmsg_.scale.z = 1.0;
      // color (randomly from color table)
      const unsigned int idx = stoi(paths_stl[i].stem().string()) % submap_colors.size(); 
      submapmsg_.color.a = submap_colors[idx](0);
      submapmsg_.color.r = submap_colors[idx](1);
      submapmsg_.color.g = submap_colors[idx](2);
      submapmsg_.color.b = submap_colors[idx](3);

      // push current submap to the array of submaps
      submaparraymsg_.markers.push_back(submapmsg_);

  } // end loop over stls

  // publish all submaps
  pubSubmaps_.publish(submaparraymsg_);

}


void Publisher::publishSubmapsAsCallback(std::unordered_map<uint64_t, Transformation> submapPoseLookup, std::unordered_map<uint64_t, SubmapList::iterator> submapLookup) 
{

  typedef se::Octree<se::Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off>, se::Res::Multi, 8> OctreeT;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  // Iterate over the octree, creating a different CUBE_LIST marker for each
  // volume size and state.
  std::map<int, visualization_msgs::Marker> markers_free;
  std::map<int, visualization_msgs::Marker> markers_occupied;
  std::map<int, visualization_msgs::Marker> markers_unknown;

  if(!submapLookup.empty()) // else do nothing
  {

  // iterate over submaps
  for (auto& it: submapLookup) {

  auto octree_ptr = (*it.second)->getOctree();

  for (auto octant_it = se::LeavesIterator<OctreeT>(octree_ptr.get()); octant_it != se::LeavesIterator<OctreeT>(); ++octant_it) {
      
      const auto octant_ptr = *octant_it;
      int node_size;
      int node_scale;

      // Differentiate between block and node processing

      // IF BLOCK
      if (octant_ptr->isBlock()) {
        // If the leaf is a block we'll need to iterate over all voxels at the current scale
        typedef typename OctreeT::BlockType BlockType;
        const Eigen::Vector3i block_coord = octant_ptr->getCoord();
        const BlockType* block_ptr = static_cast<const BlockType*>(octant_ptr);
        // Find current scale of the block leaves and their size
        const int node_scale = block_ptr->getCurrentScale();
        const int node_size  = 1 << node_scale;
        const Eigen::Vector3f twm = (*it.second)->gettWM();

        for (int x = 0; x < BlockType::getSize(); x += node_size) {
            for (int y = 0; y < BlockType::getSize(); y += node_size) {
                for (int z = 0; z < BlockType::getSize(); z += node_size) {

                  const Eigen::Vector3i node_coord = block_coord + Eigen::Vector3i(x, y, z);
                  const Eigen::Vector3f node_centre_meter = (node_coord.template cast<float>() + Eigen::Vector3f::Constant((float) node_size / 2)) * (*it.second)->getRes();
                  const auto data = block_ptr->getData(node_coord);

                  if (data.occupancy * data.weight < 0) { // FREE VOXELS
                      continue;
                  }

                  std::map<int, visualization_msgs::Marker>* markers = nullptr;
                  std::string ns;
                  std_msgs::ColorRGBA volume_color;

                  if(data.occupancy * data.weight > 0){ // OCCUPIED VOXELS
                    markers = &markers_occupied;
                    ns = "map_occupied";
                    volume_color.r = color_occupied_.x();
                    volume_color.g = color_occupied_.y();
                    volume_color.b = color_occupied_.z();
                    volume_color.a = color_occupied_.w();
                  }

                  else { // UNKNOWN VOXELS
                    markers = &markers_unknown;
                    ns = "map_unknown";
                    volume_color.r = color_unknown_.x();
                    volume_color.g = color_unknown_.y();
                    volume_color.b = color_unknown_.z();
                    volume_color.a = color_unknown_.w();
                  }

                  const int size = node_size;
                  float resolution = (*it.second)->getRes();
                  if (markers->count(size) == 0) {
                    // Initialize the Marker message for this voxel size.
                    (*markers)[size] = visualization_msgs::Marker();
                    (*markers)[size].header = header;
                    (*markers)[size].ns = ns;
                    (*markers)[size].id = size;
                    (*markers)[size].type = visualization_msgs::Marker::CUBE_LIST;
                    (*markers)[size].action = visualization_msgs::Marker::ADD;
                    (*markers)[size].pose.orientation.x = 0.0;
                    (*markers)[size].pose.orientation.y = 0.0;
                    (*markers)[size].pose.orientation.z = 0.0;
                    (*markers)[size].pose.orientation.w = 1.0;
                    (*markers)[size].scale.x = size*resolution;
                    (*markers)[size].scale.y = size*resolution;
                    (*markers)[size].scale.z = size*resolution;
                    (*markers)[size].color = volume_color;
                    (*markers)[size].lifetime = ros::Duration(0.0);
                    (*markers)[size].frame_locked = true;
                  }
                  // Append the current voxel.
                  const Eigen::Vector4d p_mp(node_centre_meter[0],node_centre_meter[1],node_centre_meter[2],1); // p wrt map (homogenous)
                  const Eigen::Matrix4d T_wm = (*it.second)->getTWM().cast<double>(); // map wrt "world", which is the kf
                  const Eigen::Matrix4d T_wf = submapPoseLookup[it.first].T(); // kf wrt world
                  const Eigen::Vector4d p_eigen = T_wf*T_wm*p_mp; // p wrt world (homogenous)
                  
                  geometry_msgs::Point p;
                  p.x = p_eigen[0];
                  p.y = p_eigen[1];
                  p.z = p_eigen[2];
                  (*markers)[size].points.push_back(p);

                } // z
            } // y
        } // x
      }
      // IF NODE
      else {
        // const auto data = static_cast<typename OctreeT::NodeType*>(octant_ptr)->getMaxData();
        // // Again decide which nodes to ignore
        // if (data.occupancy * data.weight <= 0) {
        //     continue;
        // }
        // node_size = static_cast<typename OctreeT::NodeType*>(octant_ptr)->getSize();
        // // Since we don't care about the node scale, just set it to a number that will result in
        // // a gray color when saving the mesh.
        // node_scale = 7;

        // // TO BE REPLACED WITH RVIZ MARKERS
        
        // // Get the coordinates of the octant vertices.
        // Eigen::Vector3f node_corners[8];
        // const Eigen::Vector3i node_coord = octant_ptr->getCoord();
        // node_corners[0] = node_coord.cast<float>();
        // node_corners[1] = (node_coord + Eigen::Vector3i(node_size, 0, 0)).cast<float>();
        // node_corners[2] = (node_coord + Eigen::Vector3i(0, node_size, 0)).cast<float>();
        // node_corners[3] = (node_coord + Eigen::Vector3i(node_size, node_size, 0)).cast<float>();
        // node_corners[4] = (node_coord + Eigen::Vector3i(0, 0, node_size)).cast<float>();
        // node_corners[5] = (node_coord + Eigen::Vector3i(node_size, 0, node_size)).cast<float>();
        // node_corners[6] = (node_coord + Eigen::Vector3i(0, node_size, node_size)).cast<float>();
        // node_corners[7] =
        //     (node_coord + Eigen::Vector3i(node_size, node_size, node_size)).cast<float>();

        // // The Quad::num_vertexes vertex indices to node_corners for each of the 6 faces.
        // int face_vertex_idx[6][Quad::num_vertexes] = {
        //     {0, 1, 3, 2}, {1, 5, 7, 3}, {5, 7, 6, 4}, {0, 2, 6, 4}, {0, 1, 5, 4}, {2, 3, 7, 6}};
        
        // // Create the octant faces.
        // for (int f = 0; f < 6; ++f) {
        //     mesh.emplace_back();
        //     for (size_t v = 0; v < Quad::num_vertexes; ++v) {
        //         mesh.back().vertexes[v] = node_corners[face_vertex_idx[f][v]];
        //         mesh.back().max_vertex_scale = node_scale;
        //     }
        // }
      }

      // Publish all markers.
      // for (const auto& marker : markers_free) {
      //   map_free_pub_.publish(marker.second);
      // }
      for (const auto& marker : markers_occupied) {
        map_occupied_pub_.publish(marker.second);
      }
      // for (const auto& marker : markers_unknown) {
      //   map_unknown_pub_.publish(marker.second);
      // }
  }

  }

  }
}

void Publisher::publishPathAsCallback(const ompl::geometric::PathGeometric & path)
{
  
  // TODO smooth?

  // std::cout << "\n\n(Publisher) publishing path!\n\n";

  nav_msgs::Path  msg;
  msg.header.stamp = ros::Time::now(); // se if path has timestamps
  msg.header.frame_id = "world";

  for (std::size_t idx = 0; idx <  path.getStateCount (); idx++)
  {
    // const ompl::base::SE3StateSpace::StateType *se3state =  path.getState(idx)->as<ompl::base::SE3StateSpace::StateType>();

    const ompl::base::RealVectorStateSpace::StateType *pos = path.getState(idx)->as<ompl::base::RealVectorStateSpace::StateType>();

    // const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);
    
    geometry_msgs::PoseStamped point;

    point.pose.position.x = pos->values[0];
    point.pose.position.y = pos->values[1];
    point.pose.position.z = pos->values[2];

    // point.pose.orientation.x = rot->x;
    // point.pose.orientation.y = rot->y;
    // point.pose.orientation.z = rot->z;
    // point.pose.orientation.w = rot->w;

     msg.poses.push_back(point);

  }

  pubOMPLPath_.publish( msg);
}

/*void Publisher::publishMarkerAsCallback(
      const okvis::Time & , const okvis::kinematics::Transformation & T_WT, const Eigen::Vector3d & v_W, const::Eigen::Vector3d & ) {

  okvis::kinematics::Transformation T_Wc_T = parameters_.publishing.T_Wc_W * T_WT;

// low-pass-filter
Eigen::Vector3d r ;
Eigen::Vector3d v ;
//if(target_pos.norm()<1.0e-12) {
  target_pos = T_Wc_T.r();
  target_vel = parameters_.publishing.T_Wc_W.C() * v_W;    // v_S_in_'speedsInThisFrame' or v_B_in_'speedsInThisFrame'
  r = target_pos;
  v = target_vel;
//} else {
//  target_pos = target_pos*0.2+0.8*T_Wc_T.r();
//  target_vel = target_vel*0.8+0.2*parameters_.publishing.T_Wc_W.C() * v_W;    // v_S_in_'speedsInThisFrame' or v_B_in_'speedsInThisFrame'
//  r = target_pos;
//  v = target_vel;
//}
 


  markerPoseMsg_.child_frame_id = "marker";
  markerPoseMsg_.header.frame_id = "world";
  markerPoseMsg_.header.stamp = _t;

  // fill orientation
  Eigen::Quaterniond q = T_Wc_T.q();
  markerPoseMsg_.transform.rotation.x = q.x();
  markerPoseMsg_.transform.rotation.y = q.y();
  markerPoseMsg_.transform.rotation.z = q.z();
  markerPoseMsg_.transform.rotation.w = q.w();

  // fill position

  markerPoseMsg_.transform.translation.x = r[0];
  markerPoseMsg_.transform.translation.y = r[1];
  markerPoseMsg_.transform.translation.z = r[2];

  if ((ros::Time::now() - _t).toSec() > 10.0)
    markerPoseMsg_.header.stamp = ros::Time::now();
  pubMarkerTransform_.publish(markerPoseMsg_);
  
  pubTf_.sendTransform(markerPoseMsg_);

  // also publish odometry
  markerOdometryMsg_.header.frame_id = "world";
  markerOdometryMsg_.header.stamp = _t;

  // fill orientation
  markerOdometryMsg_.pose.pose.orientation.x = q.x();
  markerOdometryMsg_.pose.pose.orientation.y = q.y();
  markerOdometryMsg_.pose.pose.orientation.z = q.z();
  markerOdometryMsg_.pose.pose.orientation.w = q.w();

  // fill position
  markerOdometryMsg_.pose.pose.position.x = r[0];
  markerOdometryMsg_.pose.pose.position.y = r[1];
  markerOdometryMsg_.pose.pose.position.z = r[2];

  // fill velocity
  // HACKED for MBZIRC

  markerOdometryMsg_.twist.twist.linear.x = v[0];
  markerOdometryMsg_.twist.twist.linear.y = v[1];
  markerOdometryMsg_.twist.twist.linear.z = v[2];

  // fill angular velocity -- ignore
  markerOdometryMsg_.twist.twist.angular.x = 0.0;
  markerOdometryMsg_.twist.twist.angular.y = 0.0;
  markerOdometryMsg_.twist.twist.angular.z = 0.0;

  // publish!!
  pubMarkerOdometry_.publish(markerOdometryMsg_);
}*/

// Set and write landmarks to file.
void Publisher::csvSaveLandmarksAsCallback(
    const okvis::Time & /*t*/, const okvis::MapPointVector & actualLandmarks,
    const okvis::MapPointVector & transferredLandmarks)
{
  okvis::MapPointVector empty;
  setPoints(actualLandmarks, empty, transferredLandmarks);
  if (csvLandmarksFile_) {
    if (csvLandmarksFile_->good()) {
      for (size_t l = 0; l < actualLandmarks.size(); ++l) {
        Eigen::Vector4d landmark = actualLandmarks.at(l).point;
        *csvLandmarksFile_ << std::setprecision(19) << actualLandmarks.at(l).id
            << ", " << std::scientific << std::setprecision(18) << landmark[0]
            << ", " << landmark[1] << ", " << landmark[2] << ", " << landmark[3]
            << ", " << actualLandmarks.at(l).quality
            // << ", " << actualLandmarks.at(l).distance
            << std::endl;
      }
    }
  }
}

// Publish the last set points.
void Publisher::publishPoints()
{
  pubPointsMatched_.publish(pointsMatched_);
  pubPointsUnmatched_.publish(pointsUnmatched_);
  pubPointsTransferred_.publish(pointsTransferred_);
}

// Set the images to be published next.
void Publisher::setImages(const std::vector<cv::Mat> & images)
{
  // copy over
  images_.resize(images.size());
  for (size_t i = 0; i < images.size(); ++i)
    images_[i] = images[i];
}

// Add a pose to the path that is published next. The path contains a maximum of
// maxPathLength poses that are published. Once the
// maximum is reached, the last pose is copied in a new path message. The rest are deleted.
void Publisher::setPath(const okvis::kinematics::Transformation &T_WS)
{
  const int maxPathLength = 1000;
  //if (path_.poses.size() >= parameters_.publishing.maxPathLength) {
  if (path_.poses.size() >= maxPathLength) {
    geometry_msgs::PoseStamped lastPose = path_.poses.back();
    path_.poses.clear();
    //path_.poses.reserve(parameters_.publishing.maxPathLength);
    path_.poses.reserve(maxPathLength);
    path_.poses.push_back(lastPose);
  }
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = _t;
  pose.header.frame_id = "world";
  //okvis::kinematics::Transformation T = parameters_.publishing.T_Wc_W*T_WS;
  okvis::kinematics::Transformation T = T_WS;

  // put the path into the origin of the selected tracked frame
  //if (parameters_.publishing.trackedBodyFrame == FrameName::S) {
    // nothing
  /*} else if (parameters_.publishing.trackedBodyFrame == FrameName::B) {
    T = T * parameters_.imu.T_BS.inverse();
  } else {
    LOG(ERROR) <<
        "Pose frame does not exist for publishing. Choose 'S' or 'B'.";
    T = T * parameters_.imu.T_BS.inverse();
  }*/
  /// \todo
  
  const Eigen::Vector3d& r = T.r();
  pose.pose.position.x = r[0];
  pose.pose.position.y = r[1];
  pose.pose.position.z = r[2];
  const Eigen::Quaterniond& q = T.q();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  path_.header.stamp = _t;
  path_.header.frame_id = "world";
  path_.poses.push_back(pose);
}

// Publish the last set images.
void Publisher::publishImages()
{
  // advertise what's been missing:
  if (images_.size() != pubImagesVector_.size()) {
    pubImagesVector_.clear();
    for (size_t i = 0; i < images_.size(); ++i) {
      std::stringstream drawingNameStream;
      drawingNameStream << "okvis_drawing_" << i;
      imageTransportVector_.push_back(image_transport::ImageTransport(*nh_));
      pubImagesVector_.push_back(
          imageTransportVector_[i].advertise(drawingNameStream.str(), 10));
    }
  }

  // publish:
  for (size_t i = 0; i < images_.size(); ++i) {
    sensor_msgs::Image msg;
    std::stringstream cameraNameStream;
    cameraNameStream << "camera_" << i;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = cameraNameStream.str();
    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8,
                           images_[i].rows, images_[i].cols,
                           images_[i].step.buf[0], images_[i].data);
    pubImagesVector_[i].publish(msg);
  }
}

// Publish the last set path.
void Publisher::publishPath()
{
  pubPath_.publish(path_);
}

// }  // namespace okvis
