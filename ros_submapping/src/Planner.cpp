#include "Planner.hpp"

Planner::Planner(SupereightInterface* se_interface_, const std::string& filename) {

  // Set preempt flag
  // preempt_plan = false;
  
  // ============ SET SEINTERFACE PTR ============  

  se_interface = se_interface_;

  // ============ SET STATE SPACE ============

  space = std::make_shared<ob::RealVectorStateSpace>(3);

  // set the bounds for the R^3 part of SE(3) (position)

  float raw_bounds[6];

  // Open the YAML file for reading
  // TODO catch exception for missing stuff in yaml

  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

  // Get the node containing map configs
  const cv::FileNode node_map = fs["map"];
  // Read map res (need it in collision function)
  se::yaml::subnode_as_float(node_map, "res", map_res);

  // Get the node containing planner configs
  const cv::FileNode node_planner = fs["planner"];

  // Read the config parameters.
  se::yaml::subnode_as_float(node_planner, "min_x", raw_bounds[0]);
  se::yaml::subnode_as_float(node_planner, "max_x", raw_bounds[1]);
  assert(raw_bounds[0] <= raw_bounds[1]);
  se::yaml::subnode_as_float(node_planner, "min_y", raw_bounds[2]);
  se::yaml::subnode_as_float(node_planner, "max_y", raw_bounds[3]);
  assert(raw_bounds[2] <= raw_bounds[3]);
  se::yaml::subnode_as_float(node_planner, "min_z", raw_bounds[4]);
  se::yaml::subnode_as_float(node_planner, "max_z", raw_bounds[5]);
  assert(raw_bounds[4] <= raw_bounds[5]);

  // set collision function radius
  se::yaml::subnode_as_float(node_planner, "mav_radius", mav_radius);
  assert(mav_radius >= 0);

  std::cout << "\n\nPlanner bounds: ";

  for (int i = 0; i < 6; i++)
  {
    std::cout << raw_bounds[i] << "  ";
  }

  std::cout << "\n\n";

  std::cout << "\n\nMAV radius in planner: " << mav_radius << "\n\n";

  ob::RealVectorBounds bounds(3);

  // set bounds
  bounds.setLow(0, raw_bounds[0]);
  bounds.setHigh(0,raw_bounds[1]);
  bounds.setLow(1,raw_bounds[2]);
  bounds.setHigh(1,raw_bounds[3]);
  bounds.setLow(2,raw_bounds[4]);
  bounds.setHigh(2,raw_bounds[5]);

  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // ============ SET SIMPLESETUP ============

  ss = std::make_shared<og::SimpleSetup>(space);

  // create start state
  ob::ScopedState<ob::RealVectorStateSpace> start_ompl(space);
  (*start_ompl)[0] = 0;
  (*start_ompl)[1] = 0;
  (*start_ompl)[2] = 0;

  // create goal state
  ob::ScopedState<ob::RealVectorStateSpace> goal_ompl(space);
  (*goal_ompl)[0] = 0;
  (*goal_ompl)[1] = 0;
  (*goal_ompl)[2] = 0;
  
  // set start & goal
  ss->setStartAndGoalStates(start_ompl, goal_ompl);
  
  // set collision checker
  ss->setStateValidityChecker(std::bind(&Planner::detectCollision, this, std::placeholders::_1 )); 

  // specify cost heuristic for planner.
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(ss->getSpaceInformation()));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);

  // // (optional) if we want to benchmark the planner, lets take the first (prob. crappy) solution, with this hack
  // // if we dont set this, rrtstar will keep on looking for better solutions until it runs out of time
  // only need this for rrt* /informed rrt * (the optimal ones)
  // rrt and rrt connect take the first sol by default,
  // and then you can simplify the solution 
  // ob::Cost cost(1000.0); // set super high cost treshold
  // obj->setCostThreshold(cost); // so planning stops as soon as a first solution is found

  // set Optimization objective
  ss->setOptimizationObjective(obj);
  
  // set planner
  //rrt = std::make_shared<og::InformedRRTstar>(ss->getSpaceInformation());
  rrt = std::make_shared<og::RRTConnect>(ss->getSpaceInformation());
  rrt->setRange(0.4);

  ss->setPlanner(rrt);

  // create empty path
  path = std::make_shared<og::PathGeometric>(ss->getSpaceInformation());

}

void Planner::setStart(const Eigen::Vector3d & r)
{

  start = r;

}

void Planner::setGoal(const Eigen::Vector3d & r)
{

  goal = r;

}

// pass by value to avoid dangling reference
bool Planner::plan(const Eigen::Vector3d r)
{ 

  std::cout << "AAAAA \n";

  // As of now, thread is blocked if there's a running plan() thread. 
  // Should not be this way --> preempt running thread with latest
  
  // TODO:
  // preempt any other running planning thread, and wait for it to free the mutex.
  // warning: preempt is not immediate, so we should still wait for the thread to free the mutex.
  // Almost there, but ptc does not work inside the planner! planner does not check for termination condition!

  // preempt_plan = true;
  std::unique_lock<std::mutex> lk(planMutex);
  // flag is lowered here, as soon as the thread takes control
  // preempt_plan = false;

  // need this later, for a hack in collision detector
  start_fixed = start;

  // set the fixed lookups for the collision checking func
  se_interface->fixReadLookups();

  if (se_interface->submapLookup_read.empty() || se_interface->hashTable_read.empty()) {
    std::cout << "Planner failed. No maps yet. \n";
    return false;
  }

  std::cout << "\n\nPlanning from: " 
  << start[0] << " " 
  << start[1] << " " 
  << start[2] << " to: " 
  << goal[0]<< " " 
  << goal[1]<< " " 
  << goal[2]<< "\n\n";

  // create ompl start state
  ob::ScopedState<ob::RealVectorStateSpace> start_ompl(space);
  (*start_ompl)[0] = start[0];
  (*start_ompl)[1] = start[1];
  (*start_ompl)[2] = start[2];

  // create ompl goal state
  ob::ScopedState<ob::RealVectorStateSpace> goal_ompl(space);
  (*goal_ompl)[0] = goal[0];
  (*goal_ompl)[1] = goal[1];
  (*goal_ompl)[2] = goal[2];

  ss->clear();

  // load current start & goal
  ss->setStartAndGoalStates(start_ompl, goal_ompl);

  // Planner termination condition. 
  // Thread terminates either when preempted by new planing query
  // or when too much time has passed.
  start_time = std::chrono::steady_clock::now();
  // ob::PlannerTerminationCondition ptc(std::bind(&Planner::terminatePlanner,this));
  // ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(2.0);

  ob::PlannerStatus solved = ss->solve(10.0);
  //ob::PlannerStatus solved = ss->solve(*ptc);
  
  if (solved) {

  // get optimal path
  *path = ss->getSolutionPath();

  og::PathSimplifier path_simp(ss->getSpaceInformation()); // path simplifier
  path_simp.simplify(*path,0.05); // "simplify" the path
  path_simp.smoothBSpline(*path); // smooth the path
  
  // send path to visualizer
  if(pathCallback_) {
    std::thread publishPath(pathCallback_, *path);
    publishPath.detach();
  }

  // TODO: REF PUBLISHER 
  // std::thread publishCtrl(ctrlPublisher, path); // publish as references for the controller
  // ctrlPublisher.detach();

  return true; 
  }

  lk.unlock();
  
  return false; // if not solved
 
}

bool Planner::plan() {
  return plan(goal);
}

void Planner::processState(const okvis::State& state, const okvis::TrackingState& trackingstate)
{

  start = state.T_WS.r();

}


bool Planner::detectCollision(const ompl::base::State *state) 
{

  // buttami!
  std::this_thread::sleep_for (std::chrono::milliseconds(300));

  const ompl::base::RealVectorStateSpace::StateType *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  // the voxel we want to query (wrt world frame, homogenous)
  Eigen::Vector4d r(pos->values[0],pos->values[1],pos->values[2],1);

  // Hack: when e.g. camera moves backwards we move out of the map -> this means that start
  // state is always invalid! We can avoid this by arbitrarily saying that if state is real close to 
  // start -> is free. Adds a bit of overhead (but maybe also saves some time). Comment out when benchmarking
  if((r.head<3>() - start_fixed).norm() < 0.5) return true;

  // check occ inside a sphere around the drone 
  // to avoid missing inbetween voxels, should check 
  // with a step that is <= res! should be = sqrt(2) * res
  // to be extra safe, since each submap's grid is rotated wrt 
  // world grid, but let's do = res for efficiency

  for (float z = -mav_radius; z <= mav_radius; z += map_res)
  {
    for (float y = -mav_radius; y <= mav_radius; y += map_res)
    {
      for (float x = -mav_radius; x <= mav_radius; x += map_res)
      {
        if((std::pow(x,2) + std::pow(y,2) + std::pow(z,2)) > std::pow(mav_radius,2)) continue; // skip if point is outside radius

        // CHECK OCCUPANCY AMONG LOCAL SUBMAPS:

        // add offset to r 
        Eigen::Vector4d r_new = r;
        r_new(0) += x;
        r_new(1) += y;
        r_new(2) += z;

        // its respective truncated coordinates (the 1x1x1 box it's in)
        Eigen::Vector3i box_coord;
        for (int i = 0 ; i < 3 ; i++)
        {
          box_coord(i) = (int)(floor(r_new(i)));
        }

        // need this to avg occupancy
        double tot_occupancy = 0; 
        double tot_weight = 0;
        
        // iterate over submap ids (only the ones that contain current state!)
        // if not in any submap -> return false
        if (!se_interface->hashTable_read.count(box_coord)) return false;
        for (auto& id: se_interface->hashTable_read[box_coord]) {

          // transform state coords to check from world to map frame
          const Eigen::Matrix4d T_wf = se_interface->submapPoseLookup_read[id].T(); // kf wrt world
          const Eigen::Vector4d r_map_hom = T_wf.inverse() * r_new;// state coordinates (homogenous) in map frame
          const Eigen::Vector3f r_map = r_map_hom.head<3>().cast<float>(); // take first 3 elems and cast to float
          
          // if voxel belongs to current submap -> add occupancy
          if(!se_interface->submapLookup_read.count(id)) continue;
          if((*se_interface->submapLookup_read[id])->contains(r_map))
          {
            auto data = (*se_interface->submapLookup_read[id])->getData(r_map);
            tot_occupancy += data.occupancy * data.weight;
            tot_weight += data.weight;
          }
        } 
        
        // when done iterating over submaps, check total occupancy (weighted average)
        if(tot_weight = 0) return false;
        if(tot_occupancy/tot_weight > -10) return false; 
        
      } // x
    } // y
  } // z

  // if we reach this point, it means every point in the sphere is free
  return true;

}

// bool Planner::terminatePlanner(){

//   // if there's a new planning thread or too much time has elapsed
//   if (preempt_plan)
//   {
//     std::cout << "\n\nNew query: planning preempted! \n\n";
//     return true;
//   }

//   if (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time).count() > 5.0)
//   {
//     std::cout << "\n\nAbort planning: taking too long! \n\n";
//     return true;
//   }

//   return false;

// }