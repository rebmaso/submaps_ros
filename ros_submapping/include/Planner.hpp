#ifndef INCLUDE_PLANNER_HPP_
#define INCLUDE_PLANNER_HPP_

#include <thread>
#include <chrono>
#include <atomic>
#include <fstream>
#include <functional>
#include <iostream>
#include <vector>
#include <memory>
#include <stdlib.h>

// if you want to use omplapp:
// #include <ompl/control/planners/kpiece/KPIECE1.h>
// #include <ompl/control/planners/rrt/RRT.h>
// // #include <omplapp/apps/QuadrotorPlanning.h>
// // #include <omplapp/config.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <Eigen/Core>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <okvis/ViInterface.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Time.hpp>
#include <okvis/Measurements.hpp>
#include "SupereightInterface.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::function<void(const og::PathGeometric & path)> pathCallback;

class Planner
{
private:

  // Pointer to the object that we use to get maps.
  SupereightInterface* se_interface;

  // The state space we are planning in
  ob::StateSpacePtr space; 

  og::SimpleSetupPtr ss;

  //std::shared_ptr<og::InformedRRTstar> rrt;
  std::shared_ptr<og::RRTConnect> rrt;

  std::shared_ptr<og::PathGeometric> path;

  // Updated continuously by SLAM callback
  Eigen::Vector3d start;

  // Is set at each planning query.
  Eigen::Vector3d start_fixed; 

  // Is set at each planning query.
  Eigen::Vector3d goal;

  // The closest we can get to obstacles.
  float mav_radius;

  // External visualizer 
  pathCallback pathCallback_;

  // Need this to allow only 1 plan() at a time
  std::mutex planMutex;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  
  Planner() = delete;

  /**
   * @brief      Sets the OMPL configs.
   *
   * @param[in]  se_interface_          Ponter to SupereightInterface object that we use to access maps.
   * @param[in]  filename  Config file that we use to get planner bounds.
   *
   */
  Planner(SupereightInterface* se_interface_, const std::string& filename);

  ~Planner() = default;

  /**
   * @brief      Sets the start state.
   *
   * @param[in]  r The start state.          
   *
   */
  void setStart(const Eigen::Vector3d & r);

  /**
   * @brief      Sets the goal state.
   *
   * @param[in]  r The goal state.          
   *
   */
  void setGoal(const Eigen::Vector3d & r);
  
  /**
   * @brief      Plans path from current start to goal
   *
   */
  bool plan();

  void setPathCallback(const pathCallback &pathCallback) { pathCallback_ = pathCallback; }

  /**
   * @brief      Gets state update from Okvis update
   *
   * @param[in]  state          Okvis state.
   * @param[in]  trackingstate  Okvis tracking state.
   *
   */
  void processState(const okvis::State& state, const okvis::TrackingState& trackingstate);

  /**
   * @brief     OMPL collision detector. Checks among local submaps using spatial hash table.
   *
   * @param[in]  state          Queried state.
   *
   */
  bool detectCollision(const ompl::base::State *state);

};


#endif /* INCLUDE_PLANNER_HPP_ */