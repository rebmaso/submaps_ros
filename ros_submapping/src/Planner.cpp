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

  // gazebo garching
  bounds.setLow(0,-3);
  bounds.setHigh(0,10);
  bounds.setLow(1,-3);
  bounds.setHigh(1,6);
  bounds.setLow(2,-1);
  bounds.setHigh(2,5);

  // set the bounds you just chose
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // ============ SET SIMPLESETUP ============

  ss = std::make_shared<og::SimpleSetup>(space);

  // create a start state
  start = std::make_shared<ob::ScopedState<ob::RealVectorStateSpace>>(space);
  (**start)[0] = 0;
  (**start)[1] = 0;
  (**start)[2] = 0;
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
  
  // create a goal state
  goal = std::make_shared<ob::ScopedState<ob::RealVectorStateSpace>>(space);
  (**goal)[0] = 0;
  (**goal)[1] = 0;
  (**goal)[2] = 0;
  // (*goal)->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
  
  // set start & goal
  ss->setStartAndGoalStates((*start), (*goal));
  
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
  rrt = std::make_shared<og::InformedRRTstar>(ss->getSpaceInformation());
  rrt->setRange(0.2);

  ss->setPlanner(rrt);
  
  // create empty path
  path = std::make_shared<og::PathGeometric>(ss->getSpaceInformation());

  //started = false;

  std::cout << "\n\nPlanner initialized...\n\n";

}

void Planner::setStart(const Eigen::Vector3d & r)
{
  // position
  // (*start)->setXYZ(r[0],r[1],r[2]);
  for (int i = 0; i < 3; i++)
  {
    (**start)[i] = r[i];
  }

  // orientation
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->x = q.x();
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->y = q.y();
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->z = q.z();
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->w = q.w();
  
  // std::cout << "(planner) new start set to: " << r[0] << " " << r[1] << " " << r[2] << std::endl;
}

void Planner::setGoal(const Eigen::Vector3d & r)
{
  // (*goal)->setXYZ(r[0],r[1],r[2]);
  for (int i = 0; i < 3; i++)
  {
    (**goal)[i] = r[i];
  }


  // (*goal)->as<ob::SO3StateSpace::StateType>(1)->x = q.x();
  // (*goal)->as<ob::SO3StateSpace::StateType>(1)->y = q.y();
  // (*goal)->as<ob::SO3StateSpace::StateType>(1)->z = q.z();
  // (*goal)->as<ob::SO3StateSpace::StateType>(1)->w = q.w();

  //started = true;

  //  plan(); // should plan whenever I set a new goal I get a new goal (returns after plan() is done)
}

bool Planner::plan()
{ 
  // this condition is needed to avoid planning when we have not assigned a goal yet (at startup)
  // but new maps are still added because e.g. the drone is flying around manually
  // when we set the first goal with setGoal() -> start = true
  // if(!started) return true;

  // set the fixed lookups for the collision checking func

  if (se_interface->submapLookup_.empty()) {
    std::cout << "Planner failed. No maps yet. \n";
    return false;
  }

  se_interface->fixReadLookups();

  std::cout << "\n\nPlanning from: " 
  << (**start)[0] << " " 
  << (**start)[1] << " " 
  << (**start)[2] << " to: " 
  << (**goal)[0]<< " " 
  << (**goal)[1]<< " " 
  << (**goal)[2]<< "\n\n";

  ss->clear();

  // load current start & goal
  ss->setStartAndGoalStates((*start), (*goal));

  ob::PlannerStatus solved = ss->solve(2.0);

  if (solved) {

  // get optimal path
  // path.reset(&(ss->getSolutionPath())); // which way is correct?
  *path = ss->getSolutionPath();
  
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

  okvis::kinematics::Transformation T = state.T_WS;

  Eigen::Quaterniond q = T.q();
  Eigen::Vector3d r = T.r();

  // set start state:
  
  // position
  for (int i = 0; i < 3; i++)
  {
    (**start)[i] = r[i];
  }
  // orientation
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->x = q.x();
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->y = q.y();
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->z = q.z();
  // (*start)->as<ob::SO3StateSpace::StateType>(1)->w = q.w();

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

  if(se_interface->submapLookup_read.empty()) return false;

  const ompl::base::RealVectorStateSpace::StateType *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

  // the voxel we want to query (wrt world frame, homogenous)
  Eigen::Vector4d r(pos->values[0],pos->values[1],pos->values[2],1);

  // check occ inside a sphere around the drone 
  // very sparse 
  // lowest res is 1 voxel = 0.2m
  const float rad = 0.2; // radius of the sphere

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

        if(tot_occupancy >= 0) {
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