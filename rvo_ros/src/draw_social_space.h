#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Header.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/WorldState.h"
#include <string>
#include "../rvo_lib/nav_rvo.h"
#include "rvo_ros/SetGoals.h"
#include "gazebo_msgs/WorldState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <math.h> 
#include <time.h>

struct SocialArea
{
  double variance_front;
  double variance_side;
  double variance_back;
  double cx;
  double cy;
  double theta;
};

const int num_max = 30;
int num_agent = 0;
int copy_num_agent = 1;
bool arrived = false;

ros::Publisher __pub_social;


void rvo_stateCallback(const gazebo_msgs::WorldState::ConstPtr& sub_msg);