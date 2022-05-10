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

  SupereightInterface* se_interface;

  ob::StateSpacePtr space; // the state space we are planning in

  og::SimpleSetupPtr ss;

  //std::shared_ptr<og::InformedRRTstar> rrt;
  std::shared_ptr<og::RRTConnect> rrt;

  std::shared_ptr<og::PathGeometric> path;

  // std::shared_ptr<ob::ScopedState<ob::RealVectorStateSpace>> start;

  // std::shared_ptr<ob::ScopedState<ob::RealVectorStateSpace>> goal;

  Eigen::Vector3d start; // updated continuously by slam callback

  Eigen::Vector3d start_fixed; // is the one that was used at last planning query

  Eigen::Vector3d goal; // updated at each planning query

  float mav_radius;

  pathCallback pathCallback_; // external visualizer (it's in Publisher)

  //bool started; // a flag that is needed to avoid unnecessary planning at startup

  std::mutex planMutex; // need this to allow only 1 plan() at a time

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // to avoid alignment issues, just in case
  
  Planner() = delete;

  // sets planner config. pass the supereightinterface svc handle from main + config file for planner bounds
  Planner(SupereightInterface* se_interface_, const std::string& filename);

  ~Planner() = default;

  void setStart(const Eigen::Vector3d & r);

  void setGoal(const Eigen::Vector3d & r);
  
  // plans path from start to goal
  bool plan();

  void setPathCallback(const pathCallback &pathCallback) { pathCallback_ = pathCallback; }

  // void updateStartState(const Eigen::Vector3f & r);

  void processState(const okvis::State& state, const okvis::TrackingState& trackingstate);

  bool detectCollision(const ompl::base::State *state);

};


#endif /* INCLUDE_PLANNER_HPP_ */