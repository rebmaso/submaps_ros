#include "Planner.hpp"

Planner::Planner(SupereightInterface* se_interface_) {

  // ============ SET SEINTERFACE PTR ============  

  se_interface = se_interface_;

  // ============ SET STATE SPACE ============

  space = std::make_shared<ob::RealVectorStateSpace>(3);

  // set the bounds for the R^3 part of SE(3) (position)
  ob::RealVectorBounds bounds(3);

  // important: define here the box inside which you intend to plan
  bounds.setLow(0,-1);
  bounds.setHigh(0,25);
  bounds.setLow(1,-5);
  bounds.setHigh(1,4);
  bounds.setLow(2,-4);
  bounds.setHigh(2,4);

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
  ss->setStateValidityChecker(std::bind(&SupereightInterface::detectCollision, se_interface, std::placeholders::_1 )); 

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
  se_interface->fixReadLookups();

  std::cout << "\n\n(Planner) planning from: " 
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