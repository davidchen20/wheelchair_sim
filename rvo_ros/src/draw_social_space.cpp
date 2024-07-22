#include "draw_social_space.h"

uint64_t seq = 0;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "rvo_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/obstacles_velocity", 100, rvo_stateCallback);

    __pub_social = n.advertise<visualization_msgs::MarkerArray>("social_area", 1000);
    
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void rvo_stateCallback(const gazebo_msgs::WorldState::ConstPtr &sub_msg)
{
    visualization_msgs::MarkerArray mk_list;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "social_area";
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.action = visualization_msgs::Marker::ADD;
    mk.lifetime = ros::Duration(0.5);
    mk.scale.x = 0.05;

    mk.color.r = 1.0;
    mk.color.g = 0.64;
    mk.color.b = 0.0;
    mk.color.a = 1;

    auto models_name = sub_msg->name;
    int total_num = models_name.size();

    for(int i = 0; i < total_num; i++){
        geometry_msgs::Pose agent_pose = sub_msg->pose[i];
        geometry_msgs::Twist agent_velocity = sub_msg->twist[i];
        mk.id = i;

        // if(agent_velocity.linear.y !=0 || agent_velocity.linear.x != 0){
        double vel_ang = fmod(atan2(agent_velocity.linear.y, agent_velocity.linear.x) + 2 * M_PI,  2 * M_PI);
        // }

        // std::cout<<"The velocity angle is "<<vel_ang<<std::endl;

        double ray_ang = 0;
        double delta_ang = M_PI / 30;
        double cons = 0.354163;

        double front_coeff = 1.0;
        double side_coeff = 0.4; // 2/3
        double rear_coeff = 0.25; // 0.5
        double safety_dist = 0.5;
        double velocity_x = agent_velocity.linear.x;
        double velocity_y = agent_velocity.linear.y;

        double velocity_magnitude = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

        double variance_front = std::max(0.5, front_coeff * velocity_magnitude);
        double variance_side = side_coeff * variance_front;
        double variance_rear = rear_coeff * variance_front;

        while (ray_ang <= 2 * M_PI){
            int flag = int(ray_ang / (M_PI / 2));

            double prev_variance;
            double next_variance;

            if (flag == 0){
                prev_variance = variance_front;
                next_variance = variance_side;
            }
            else if( flag == 1){
                prev_variance = variance_rear;
                next_variance = variance_side;
            }
            else if( flag == 2){
                prev_variance = variance_rear;
                next_variance = variance_side;
            }
            else{
                prev_variance = variance_front;
                next_variance = variance_side;
            }

            double dist = sqrt(cons / ((cos(ray_ang) *cos(ray_ang) / (2 * prev_variance)) + (sin(ray_ang) *sin(ray_ang) / (2 * next_variance))));

            // double total_angle = fmod((ray_ang + vel_ang) , (2 * M_PI));
            double total_angle = ray_ang + vel_ang;
            // ROS_INFO_STREAM(total_angle);

            double x_offset = dist * cos(total_angle);
            double y_offset = dist * sin(total_angle);

            geometry_msgs::Point edge_point;

            edge_point.x = agent_pose.position.x + x_offset;
            edge_point.y = agent_pose.position.y + y_offset;
            edge_point.z = 0;
            // std::cout<<"ZBCHECK_dsp: "<<"edge_point.x, edge_point.y: "<<edge_point.x<<", "<<edge_point.y<<std::endl;
            
            mk.points.push_back(edge_point);

            ray_ang += delta_ang;
        }
        mk_list.markers.push_back(mk);

        mk.points.clear();
    }

    

    __pub_social.publish(mk_list);

}
