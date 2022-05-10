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
 *  Created on: Apr 17, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Publisher.hpp
 * @brief Header file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_PUBLISHER_HPP_
#define INCLUDE_PUBLISHER_HPP_

#include <fstream>
#include <filesystem>
#include <memory>

#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#pragma GCC diagnostic pop
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>

#include <okvis/ViInterface.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Time.hpp>
#include <okvis/Measurements.hpp>

#include "SupereightInterface.hpp"

#include <pcl/console/parse.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
// #include <pcl/memory.h>  // for pcl::make_shared

#include <boost/filesystem.hpp>  // for boost::filesystem::path
#include <boost/algorithm/string.hpp>  // for boost::algorithm::ends_with

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h> // to publish ompl path

// defines for pcl
#define ASCII 0
#define BINARY 1
#define BINARY_COMPRESSED 2

typedef okvis::StateId StateId;
typedef okvis::kinematics::Transformation Transformation;
typedef okvis::TrackingState TrackingState;
typedef okvis::AlignedVector<okvis::State> StateVector;
typedef okvis::State State;

/**
 * @brief This class handles the publishing to either ROS topics or files.
 */
class Publisher
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:

  /// \brief Default constructor.
  Publisher();
  ~Publisher();

  /**
   * @brief Constructor. Calls setNodeHandle().
   * @param nh The ROS node handle for publishing.
   */
  Publisher(ros::NodeHandle& nh);

  /// \name Setters
  /// \{

  /**
   * @brief Set the node handle and advertise topics.
   * @param nh The ROS node handle.
   */
  void setNodeHandle(ros::NodeHandle& nh);

  /// \brief Set an odometry output CSV file.
  /// \param csvFile The file
  bool setCsvFile(std::fstream& csvFile);
  /// \brief Set an odometry output CSV file.
  /// \param csvFileName The filename of a new file
  bool setCsvFile(std::string& csvFileName);
  /// \brief Set an odometry output CSV file.
  /// \param csvFileName The filename of a new file
  bool setCsvFile(std::string csvFileName);

  /// \brief              Set a CVS file where the landmarks will be saved to.
  /// \param csvFile      The file
  bool setLandmarksCsvFile(std::fstream& csvFile);
  /// \brief              Set a CVS file where the landmarks will be saved to.
  /// \param csvFileName  The filename of a new file
  bool setLandmarksCsvFile(std::string& csvFileName);
  /// \brief              Set a CVS file where the landmarks will be saved to.
  /// \param csvFileName  The filename of a new file
  bool setLandmarksCsvFile(std::string csvFileName);

  /**
   * @brief Set the pose message that is published next.
   * @param T_WS The pose.
   */
  void setPose(const okvis::kinematics::Transformation& T_WS);

  /**
   * @brief Set the odometry message that is published next.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and biases.
   * @param omega_S Rotational speed of Sensor frame (w.r.t. to inertial frame W)
   */
  void setOdometry(const okvis::kinematics::Transformation& T_WS,
                   const okvis::SpeedAndBiases& speedAndBiases,
                   const Eigen::Vector3d& omega_S);

  /// \brief Set the parameters
  /// @param parameters The parameters.
  /*void setParameters(const okvis::VioParameters & parameters){
    parameters_ = parameters;
    if(parameters_.publishing.referenceFrame == FrameName::G) {
      if((parameters_.publishing.T_Wc_W.T() - kinematics::Transformation::Identity().T()).norm()>1.0e-9){
         LOG(WARNING) << "T_Wc_W not identity, but will be overridden, as pose relative to GPS frame requested";
      }
      parameters_.publishing.T_Wc_W.setIdentity();
    }
  }*/

  /**
   * @brief Set the points that are published next.
   * @param pointsMatched Vector of 3D points that have been matched with existing landmarks.
   * @param pointsUnmatched Vector of 3D points that were not matched with existing landmarks.
   * @param pointsTransferred Vector of landmarks that have been marginalised out.
   */
  void setPoints(const okvis::MapPointVector& pointsMatched,
                 const okvis::MapPointVector& pointsUnmatched,
                 const okvis::MapPointVector& pointsTransferred);

  /// @brief Set the time for the next message to be published.
  void setTime(const okvis::Time& t)
  {
    _t = ros::Time(t.sec, t.nsec);
  }

  /// @brief Set the images to be published next.
  void setImages(const std::vector<cv::Mat> & images);

  /// @brief Add a pose to the path that is published next. The path contains a maximum of
  ///        \e pathLength_ (change with setPathLength)poses that are published. Once the
  ///        maximum is reached, the last pose is copied in a new path message. The rest are deleted.
  void setPath(const okvis::kinematics::Transformation& T_WS);

  /**
   * @brief set the meshes directory where the submaps are.
   */
  void setMeshesPath(std::string meshesDir);

  /**
   * @brief set T_SC to visualize camera frustum for keyframes
  */
  void setT_SC(const Eigen::Matrix4d & T_SC) {
    T_SC_ = T_SC;
    Eigen::Matrix3d R_SC = T_SC_.block<3,3>(0,0);
    Eigen::Matrix3d R;
    R << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    q_sc_ = Eigen::Quaterniond(R_SC * R);}

  /// \}
  /// \name Publish
  /// \{

  /// \brief Publish the pose.
  void publishPose();
  /// \brief Publish the T_WS transform.
  void publishTransform();

  void processState(const State& state, const TrackingState & trackingState);

  /**
   * @brief Set and publish pose.
   * @remark This can be registered with the VioInterface.
   * @param t     Timestamp of pose.
   * @param T_WS  The pose.
   */
  void publishStateAsCallback(const okvis::Time & t,
                              const okvis::kinematics::Transformation & T_WS);

  /**
   * @brief Set and publish full state.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp of state.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and IMU biases.
   * @param omega_S Rotational speed of the sensor frame.
   */
  void publishFullStateAsCallback(
      const okvis::Time & t, const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & omega_S);

  /**
   * @brief Set and publish landmarks.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp.
   * @param actualLandmarks Landmarks.
   * @param transferredLandmarks Landmarks that were marginalised out.
   */
  void publishLandmarksAsCallback(
      const okvis::Time & t, const okvis::MapPointVector & actualLandmarks,
      const okvis::MapPointVector & transferredLandmarks);

  /**
   * @brief publish submaps as meshes.
   * @param  submapPoseLookup_ the lookuptable with submaps (kf) poses.
   */
  void publishSubmapMeshesAsCallback(std::unordered_map<uint64_t, Transformation> submapPoseLookup);

  /**
   * @brief publish submaps.
   * @param  submapPoseLookup_ the lookuptable with submaps (kf) poses.
   */
  void publishSubmapsAsCallback(std::unordered_map<uint64_t, Transformation> submapPoseLookup, std::unordered_map<uint64_t, SubmapList::iterator> submapLookup);

  /**
   * @brief publish keyframe states.
   */
  void publishKeyframesAsCallback(
    const State &latestState, const TrackingState &latestTrackingState,
    std::shared_ptr<const okvis::AlignedVector<State>> keyframeStates);

  /**
   * @brief publish path computed by ompl.
   */
  void publishPathAsCallback(const ompl::geometric::PathGeometric & path);

  /**
   * @brief Set and publish tracked marker.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp.
   * @param T_WT marker pose.
   */
  //void publishMarkerAsCallback(
  //    const okvis::Time & t, const okvis::kinematics::Transformation & T_WT, const Eigen::Vector3d & v_W, const::Eigen::Vector3d & omega_W);

  /**
   * @brief Set and write full state to CSV file.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp of state.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and IMU biases.
   * @param omega_S Rotational speed of the sensor frame.
   */
  void csvSaveFullStateAsCallback(
      const okvis::Time & t, const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & omega_S);

  /**
   * @brief Set and write full state including camera extrinsics to file.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp of state.
   * @param T_WS The pose.
   * @param speedAndBiases The speeds and IMU biases.
   * @param omega_S Rotation speed of the sensor frame.
   * @param extrinsics Camera extrinsics.
   */
  void csvSaveFullStateWithExtrinsicsAsCallback(
      const okvis::Time & t,
      const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & omega_S,
      const std::vector<okvis::kinematics::Transformation,
          Eigen::aligned_allocator<okvis::kinematics::Transformation> > & extrinsics);

  /**
   * @brief Set and write landmarks to file.
   * @remark This can be registered with the VioInterface.
   * @param t Timestamp.
   * @param actualLandmarks Landmarks.
   * @param transferredLandmarks Landmarks that were marginalised out.
   */
  void csvSaveLandmarksAsCallback(
      const okvis::Time & t, const okvis::MapPointVector & actualLandmarks,
      const okvis::MapPointVector & transferredLandmarks);

  /// @brief Publish the last set odometry.
  void publishOdometry();
  /// @brief Publish the last set points.
  void publishPoints();
  /// @brief Publish the last set images.
  void publishImages();
  /// @brief Publish the last set path.
  void publishPath();

  /// @}

 private:

  /// @brief Write CSV header.
  bool writeCsvDescription();
  /// @brief Write CSV header for landmarks file.
  bool writeLandmarksCsvDescription();

  /// @name Node and subscriber related
  /// @{

  ros::NodeHandle* nh_; ///< The node handle.
  tf::TransformBroadcaster pubTf_;  ///< The transform broadcaster.
  // ros::Publisher pubPointsMatched_; ///< The publisher for matched points.
  // ros::Publisher pubPointsUnmatched_; ///< The publisher for unmatched points.
  // ros::Publisher pubPointsTransferred_; ///< The publisher for transferred/marginalised points.
  ros::Publisher pubObometry_;  ///< The publisher for the odometry.
  ros::Publisher pubPath_;  ///< The publisher for the path.
  ros::Publisher pubTransform_; ///< The publisher for the transform.
  ros::Publisher pubMarkerTransform_; ///< The publisher for the transform.
  ros::Publisher pubMarkerOdometry_; ///< The publisher for the transform.
  ros::Publisher pubMesh_; ///< The publisher for a robot / camera mesh.
  
  // ==========================

  ros::Publisher pubKeyframes_; ///< The publisher for keyframe states
  ros::Publisher pubSubmaps_; ///< The publisher for se submaps
  std::string meshesDir_; ///< The directory where submaps are
  ros::Publisher pubOMPLPath_; ///< The publisher for OMPL-computed path

  // Block publishers
  ros::Publisher map_free_pub_;
  ros::Publisher map_occupied_pub_;
  ros::Publisher map_unknown_pub_;

  // Block visualization colors
  const Eigen::Vector4f color_occupied_ = Eigen::Vector4f(1.0, 0.0, 0.0, 1.0);
  const Eigen::Vector4f color_free_ = Eigen::Vector4f(0.0, 1.0, 0.0, 0.5);
  const Eigen::Vector4f color_unknown_ = Eigen::Vector4f(0.0, 0.0, 1.0, 0.7);

  // Submap meshes colors

  std::vector<Eigen::Vector4f> submap_colors = {
    Eigen::Vector4f(0.8, 0, 0.0, 1.0),
    Eigen::Vector4f(0.8, 0, 0.5, 0),
    Eigen::Vector4f(0.8, 1, 0, 0),
    Eigen::Vector4f(0.8, 0, 0.75, 0.75),
    Eigen::Vector4f(0.8, 0.75, 0.0, 0.75),
    Eigen::Vector4f(0.8, 0.75, 0.75, 0.0),
    Eigen::Vector4f(0.8, 0.25, 0.25, 0.25)
  };

  // =========================

  std::vector<image_transport::Publisher> pubImagesVector_; ///< The publisher for the images.
  std::vector<image_transport::ImageTransport> imageTransportVector_; ///< The image transporters.

  /// @}
  /// @name To be published
  /// @{

  ros::Time _t; ///< Header timestamp.
  geometry_msgs::TransformStamped poseMsg_; ///< Pose message.
  geometry_msgs::TransformStamped markerPoseMsg_; ///< Pose message.
  nav_msgs::Odometry odometryMsg_;  ///< Odometry message.
  nav_msgs::Odometry markerOdometryMsg_; ///< Marker odometry message.
  okvis::MapPointVector pointsMatched2_;  ///< Matched points vector.
  pcl::PointCloud<pcl::PointXYZRGB> pointsMatched_; ///< Point cloud for matched points.
  pcl::PointCloud<pcl::PointXYZRGB> pointsUnmatched_; ///< Point cloud for unmatched points.
  pcl::PointCloud<pcl::PointXYZRGB> pointsTransferred_; ///< Point cloud for transferred/marginalised points.
  std::vector<cv::Mat> images_; ///< The images.
  nav_msgs::Path path_; ///< The path message.
  visualization_msgs::Marker meshMsg_; ///< Mesh message.

  /// @}

  ros::Time lastOdometryTime_;  ///< Timestamp of the last broadcasted transform. (publishPose())
  ros::Time lastOdometryTime2_; ///< Timestamp of the last published odometry message. (publishOdometry())
  ros::Time lastTransfromTime_; ///< Timestamp of the last published transform. (publishTransform())

  //okvis::VioParameters parameters_; ///< All the parameters including publishing options.

  uint32_t ctr2_; ///< The counter for the amount of transferred points. Used for the seq parameter in the header.

  std::shared_ptr<std::fstream> csvFile_; ///< CSV file to save state in.
  std::shared_ptr<std::fstream> csvLandmarksFile_;  ///< CSV file to save landmarks in.

  Eigen::Matrix4d T_SC_; // Tf from Sensor to Camera
  Eigen::Quaterniond q_sc_;

};

#endif /* INCLUDE_PUBLISHER_HPP_ */
