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
#include <ompl/geometric/SimpleSetup.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <okvis/ViInterface.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Time.hpp>
#include <okvis/Measurements.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::function<void(const og::PathGeometric & path)> pathCallback;

class Planner
{
private:

  ob::StateSpacePtr space; // the state space we are planning in

  og::SimpleSetupPtr ss;

  std::shared_ptr<og::InformedRRTstar> rrt;

  std::shared_ptr<og::PathGeometric> path;

  std::shared_ptr<ob::ScopedState<ob::RealVectorStateSpace>> start;

  std::shared_ptr<ob::ScopedState<ob::RealVectorStateSpace>> goal;

  pathCallback pathCallback_; // external visualizer (it's in Publisher)

  bool started; // a flag that is needed to avoid unnecessary planning at startup

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // to avoid alignment issues, just in case
  
  Planner() = delete;

  // sets planner config. pass the supereightinterface svc handle from main
  Planner(const ob::StateValidityCheckerFn &svc);

  ~Planner() = default;

  void setStart(const Eigen::Vector3d & r);

  void setGoal(const Eigen::Vector3d & r);
  
  // plans path from start to goal
  bool plan();

  void setPathCallback(const pathCallback &pathCallback) { pathCallback_ = pathCallback; }

  void processState(const okvis::State& state, const okvis::TrackingState& trackingstate);

};


#endif /* INCLUDE_PLANNER_HPP_ */