
#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

class waypoint_publisher
{
private:
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub;
  const float DEG_2_RAD = M_PI / 180.0;
  const double delay = 0.5;
public:
  waypoint_publisher();
  ~waypoint_publisher();
  void pubwaypoint(const double & x, const double & y, const double & z, const double & yaw);
};

waypoint_publisher::waypoint_publisher() : nh("waypoint_publisher")
{

  trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "/firefly/command/trajectory", 100);

  ROS_INFO("Started waypoint_publisher.");
  
  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

}

waypoint_publisher::~waypoint_publisher()
{
}

void waypoint_publisher::pubwaypoint(const double & x, const double & y, const double & z, const double & yaw){
  
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  Eigen::Vector3d desired_position;

  desired_position << x-0.23, y, z+0.15;

  trajectory_msg.header.stamp = ros::Time::now();

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, yaw * DEG_2_RAD, &trajectory_msg);

  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x()+0.23,
           desired_position.y(),
           desired_position.z()-0.15);

  trajectory_pub.publish(trajectory_msg);

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");

  waypoint_publisher publisher;

  // // 360 spin
  // for (double i = 0; i <= 360; i=i+10)
  // { 
  //   publisher.pubwaypoint(7,0,2,i);
  // }

 // =============== GARCHING KITCHEN ===============

  // rise up 1.5m
  for (double i = 0; i <= 1.5; i=i+0.25)
  { 
    publisher.pubwaypoint(0,0,i,0);
  }

  // fwd 7m
  for (double i = 0; i <= 7; i=i+0.25)
  { 
    publisher.pubwaypoint(i,0,1.5,0);
  }
  
  // left 4m
  for (double i = 0; i <= 4; i=i+0.25)
  {
    publisher.pubwaypoint(7,i,1.5,0);
  }

  // bwd 7m
  for (double i = 7; i >= 0; i=i-0.25)
  {
    publisher.pubwaypoint(i,4,1.5,0);
  }
  
  // right 4m
  for (double i = 4; i >= 0; i=i-0.25)
  {
    publisher.pubwaypoint(0,i,1.5,0);
  }

  // fwd 7m
  for (double i = 0; i <= 7; i=i+0.25)
  { 
    publisher.pubwaypoint(i,0,1.5,0);
  }
  
  // left 4m
  for (double i = 0; i <= 4; i=i+0.25)
  {
    publisher.pubwaypoint(7,i,1.5,0);
  }

  // bwd 7m
  for (double i = 7; i >= 0; i=i-0.25)
  {
    publisher.pubwaypoint(i,4,1.5,0);
  }
  
  // right 4m
  for (double i = 4; i >= 0; i=i-0.25)
  {
    publisher.pubwaypoint(0,i,1.5,0);
  }

  // fwd 3m
  for (double i = 0; i <= 3; i=i+0.25)
  { 
    publisher.pubwaypoint(i,0,1.5,0);
  }

  // // =============== GARCHING 2 ===============

  // double x, y, z, w;

  // for (; z <= 1.5; z += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x <= 7; x += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; y <= 2; y += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x >= 0; x -= 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; y <= 4; y += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x <= 7; x += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x >= 0; x -= 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; y >= 0; y -= 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // =============== MAZE ===============

  // double x, y, z, w;

  // for (; z <= 1; z += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x <= 6; x += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; w < 90; w += 10)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; y <= 3; y += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; w < 180; w += 10)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x >= 1; x -= 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; w > 90; w -= 10)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; y <= 6; y += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; w > 0; w -= 10)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x <= 5; x += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // =============== Kitchen Dining ===============

  // double x, y, z, w;

  // for (; z <= 2; z += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; x <= 23; x += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; w <= 90; w += 10)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; y <= 11; y += 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  // for (; z >= 0.5; z -= 0.25)
  // { 
  //   publisher.pubwaypoint(x,y,z,w);
  // }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
