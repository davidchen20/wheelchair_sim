#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

using namespace Eigen;

geometry_msgs::PoseStamped start_pos;
geometry_msgs::PoseStamped target_pos;

bool get_rob_pose = false;
bool get_clicked_point = false;

void rob_poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  start_pos.header = msg->header;
  start_pos.pose= msg->pose.pose;
  get_rob_pose = true;
}

void click_pointCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
  target_pos.header = msg->header;
  target_pos.pose = msg->pose;
  get_clicked_point = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_publisher_movebase");
  ros::NodeHandle node("~");

  ros::ServiceClient get_plan = node.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  ros::Publisher global_path_pub = node.advertise<nav_msgs::Path>("/global_path", 1);
  ros::Subscriber robot_pose_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1000, rob_poseCB);
  ros::Subscriber clicked_point_sub = node.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000, click_pointCB);

  ros::Rate rate(10);

  nav_msgs::GetPlan srv;


  while (ros::ok())
  {
    if(get_clicked_point && get_rob_pose){
      srv.request.start = start_pos;
      srv.request.goal = target_pos;
      srv.request.tolerance = 0.1;

      if (get_plan.call(srv))
    {
        // Print response message
        ROS_INFO_STREAM("Got plan with " << srv.response.plan.poses.size() << " waypoints.");
        global_path_pub.publish(srv.response.plan);
        
    }
    else
    {
        ROS_ERROR("Failed to call service /move_base/make_plan");
        return 1;
    }
    }

    else{
      ROS_INFO_STREAM("Not get the points");
    }

    rate.sleep();
  }

  return 0;
}