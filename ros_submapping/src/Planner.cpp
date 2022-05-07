#include "Planner.hpp"

Planner::Planner(SupereightInterface* se_interface_) {

  // ============ SET SEINTERFACE PTR ============  

  se_interface = se_interface_;

  // ============ SET STATE SPACE ============

  space = std::make_shared<ob::RealVectorStateSpace>(3);

  // set the bounds for the R^3 part of SE(3) (position)
  ob::RealVectorBounds bounds(3);

  // important: define here the box inside which you intend to plan
  
  // // uHumans
  // bounds.setLow(0,-1);
  // bounds.setHigh(0,25);
  // bounds.setLow(1,-5);
  // bounds.setHigh(1,4);
  // bounds.setLow(2,-4);
  // bounds.setHigh(2,4);

  // // gazebo garching
  // bounds.setLow(0,-3);
  // bounds.setHigh(0,10);
  // bounds.setLow(1,-3);
  // bounds.setHigh(1,6);
  // bounds.setLow(2,-1);
  // bounds.setHigh(2,5);

  // // main_hall 
  // bounds.setLow(0,-5);
  // bounds.setHigh(0,40);
  // bounds.setLow(1,-5);
  // bounds.setHigh(1,50);
  // bounds.setLow(2,-2);
  // bounds.setHigh(2,10);

  // stairs
  bounds.setLow(0,-10);
  bounds.setHigh(0,10);
  bounds.setLow(1,-10);
  bounds.setHigh(1,25);
  bounds.setLow(2,-10);
  bounds.setHigh(2,5);

  // set the bounds you just chose
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // ============ SET SIMPLESETUP ============

  ss = std::make_shared<og::SimpleSetup>(space);

  // // create a start state
  // start = std::make_shared<ob::ScopedState<ob::RealVectorStateSpace>>(space);
  // (**start)[0] = 0;
  // (**start)[1] = 0;
  // (**start)[2] = 0;

  // create start state
  ob::ScopedState<ob::RealVectorStateSpace> start_ompl(space);
  (*start_ompl)[0] = 0;
  (*start_ompl)[1] = 0;
  (*start_ompl)[2] = 0;

  // // create a goal state
  // goal = std::make_shared<ob::ScopedState<ob::RealVectorStateSpace>>(space);
  // (**goal)[0] = 0;
  // (**goal)[1] = 0;
  // (**goal)[2] = 0;

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

  // (optional) if we want to benchmark the planner, lets take the first (prob. crappy) solution, with this hack
  // if we dont set this, rrtstar will keep on looking for better solutions until it runs out of time
  ob::Cost cost(100.0); // set super high cost treshold
  obj->setCostThreshold(cost); // so planning stops as soon as a first solution is found

  // set Optimization objective
  ss->setOptimizationObjective(obj);
  
  // set planner
  //rrt = std::make_shared<og::InformedRRTstar>(ss->getSpaceInformation());
  rrt = std::make_shared<og::RRTConnect>(ss->getSpaceInformation());
  rrt->setRange(0.4);

  ss->setPlanner(rrt);
  
  // create empty path
  path = std::make_shared<og::PathGeometric>(ss->getSpaceInformation());

  //started = false;

  std::cout << "\n\nPlanner initialized...\n\n";

}

void Planner::setStart(const Eigen::Vector3d & r)
{

  // for (int i = 0; i < 3; i++)
  // {
  //   (**start)[i] = r[i];
  // }

  start = r;

}

void Planner::setGoal(const Eigen::Vector3d & r)
{
  // for (int i = 0; i < 3; i++)
  // {
  //   (**goal)[i] = r[i];
  // }

  goal = r;

}

bool Planner::plan()
{ 
  // this condition is needed to avoid planning when we have not assigned a goal yet (at startup)
  // but new maps are still added because e.g. the drone is flying around manually
  // when we set the first goal with setGoal() -> start = true
  // if(!started) return true;

  // set the fixed lookups for the collision checking func

  // wait if other plan thread is executing
  std::unique_lock<std::mutex> lk(planMutex);

  // need this later, for a hack in collision detector
  start_fixed = start;

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

  ob::PlannerStatus solved = ss->solve(10.0);

  if (solved) {

  // get optimal path
  // path.reset(&(ss->getSolutionPath())); // which way is correct?
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
  // std::thread publishCtrl(ctrlPublisher, path); // publish as references for the mpc
  // ctrlPublisher.detach();

  return true; 
  }
  
  // std::cout << "\n\n(Planner) Planning failed! \n\n";
  return false; // if not solved
 
}

void Planner::processState(const okvis::State& state, const okvis::TrackingState& trackingstate)
{

  // Eigen::Vector3d r = state.T_WS.r();
  
  // for (int i = 0; i < 3; i++)
  // {
  //   (**start)[i] = r[i];
  // }

  start = state.T_WS.r();

}

// void Planner::updateStartState(const Eigen::Vector3f & r)
// {
//   for (int i = 0; i < 3; i++)
//   {
//     (**start)[i] = r[i];
//   }
// }

bool Planner::detectCollision(const ompl::base::State *state) 
{
  // with hash table, checking for circle around drone

  const ompl::base::RealVectorStateSpace::StateType *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  // the voxel we want to query (wrt world frame, homogenous)
  Eigen::Vector4d r(pos->values[0],pos->values[1],pos->values[2],1);

  // Hack: when e.g. camera moves backwards we move out of the map -> this means that start
  // state is always invalid! We can avoid this by arbitrarily saying that if state is real close to 
  // start -> is free. Adds a bit of overhead (but maybe also saves some time). Comment out when benchmarking
  if((r.head<3>() - start_fixed).norm() < 0.5) return true;


  // check occ inside a sphere around the drone 
  // very sparse 
  // lowest res is 1 voxel = 0.2m
  const float rad = 0.3; // radius of the sphere

  for (float z = -rad; z <= rad; z += rad/2)
  {
    for (float y = -rad; y <= rad; y += rad/2)
    {
      for (float x = -rad; x <= rad; x += rad/2)
      {
        if((std::pow(x,2) + std::pow(y,2) + std::pow(z,2)) > std::pow(rad,2)) continue; // skip if point is outside radius

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

        // we add occupancy values here
        double tot_occupancy = 0; 

        //to consider universally unmapped voxels as occupied;
        bool unmapped = true;

        // iterate over submap ids (only the ones that contain current state!)
        if (!se_interface->hashTable_read.count(box_coord)) return false;
        for (auto& id: se_interface->hashTable_read[box_coord]) {

          // transform state coords to check from world to map frame
          const Eigen::Matrix4d T_wf = se_interface->submapPoseLookup_read[id].T(); // kf wrt world
          const Eigen::Vector4d r_map_hom = T_wf.inverse() * r_new;// state coordinates (homogenous) in map frame

          const Eigen::Vector3f r_map = r_map_hom.head<3>().cast<float>(); // take first 3 elems and cast to float
          
          // if voxel belongs to current submap
          if (!se_interface->submapLookup_read.count(id)) return false;
          if((*se_interface->submapLookup_read[id])->contains(r_map))
          {
            unmapped = false;
            auto data = (*se_interface->submapLookup_read[id])->getData(r_map);
            double occupancy = data.occupancy * data.weight; // occupancy value of the 3d point
            tot_occupancy += occupancy;
          }
        } 
        
        // when done iterating over submaps, check total occupancy / check if unmapped

        if(tot_occupancy >= -20) {
          // std::cout << "\n \n OCCUPIED! \n \n";
          return false; // occupied!
        }

        // if the voxel is not found in any submap
        if(unmapped) {
          // std::cout << "\n \n UNMAPPED! \n \n"; 
          return false;
        }
        
      } // x
    } // y
  } // z

  // if we reach this point, it means every point on the circle is free
  return true;

}