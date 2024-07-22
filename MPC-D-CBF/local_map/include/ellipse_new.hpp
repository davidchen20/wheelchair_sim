#pragma once

#include <boost/circular_buffer.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/Min_ellipse_2.h>
#include <CGAL/Min_ellipse_2_traits_2.h>
#include <CGAL/Exact_rational.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_core/GridMap.hpp>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/Point.h" //zbedit2

using Obstacle = std::vector<Eigen::Array2i>;

struct Ellipse
{
  double semimajor; // Length of semi-major axis //original
  double semiminor; // Length of semi-minor axis //original
  double cx;        // x-coordinate of center
  double cy;        // y-coordinate of center
  double theta;     // Rotation angle
  double velocity; //zbedit2
  double sigma_h; //zbedit2
  double sigma_s; //zbedit2
  double sigma_r; //zbedit2
  double orient_rad; //zbedit2
  int label;        // label of obstacle 
  double variance;  // variance of ab^1.3
  ros::Time time_stamp = ros::Time::now(); //zbedit2
};

float localmap_x_size, localmap_y_size;
float resolution = 0.05, _inv_resolution = 1.0 / resolution;

using namespace std;
using namespace Eigen;

double calculate_dis(const Ellipse &e1, const Ellipse &e2) //zbattn
{
  return sqrt(pow(e1.cx - e2.cx, 2) + pow(e1.cy - e2.cy, 2));
}

/*!MSE function using CGAL tools.
 * @reference: https://doc.cgal.org/latest/Bounding_volumes/classCGAL_1_1Min__ellipse__2.html
 */
typedef CGAL::Exact_rational NT;
typedef CGAL::Cartesian<NT> K;
typedef CGAL::Point_2<K> Point;
typedef CGAL::Min_ellipse_2_traits_2<K> Traits;
typedef CGAL::Min_ellipse_2<Traits> Min_ellipse;
bool get_min_area_ellipse(const std::vector<Point> &pts, Ellipse &ellipse)
{
  Min_ellipse me(pts.begin(), pts.end(), true);
  if (me.is_degenerate())
    return false;
  // r*x^2 + s*y^2 + t*x*y + u*x + v*y + w = 0
  double r, s, t, u, v, w;
  me.ellipse().double_coefficients(r, s, t, u, v, w);
  // double a = -std::sqrt(2 * (r * v * v + s * u * u - t * u * v + (t * t - 4 * r * s) * w) *
  //                       ((r + s) + std::sqrt((r - s) * (r - s) + t * t))) / 
  //            (t * t - 4 * r * s); //original
  // double b = -std::sqrt(2 * (r * v * v + s * u * u - t * u * v + (t * t - 4 * r * s) * w) *
  //                       ((r + s) - std::sqrt((r - s) * (r - s) + t * t))) / 
  //            (t * t - 4 * r * s); //original
  double velocity = 0;
  double orient_rad = 0;
  double sigma_h = 0.5; //zbedit2 //Sigma value assuming velocity is 0
  double sigma_s = (2.0/3.0) * sigma_h; //zbedit2
  double sigma_r = 0.5 * sigma_h; //zbedit2
  double cx = (2 * s * u - t * v) / (t * t - 4 * r * s);
  double cy = (2 * r * v - t * u) / (t * t - 4 * r * s);
  double theta;
  if (t != 0)
    theta = std::atan(1 / t * (s - r - std::sqrt((r - s) * (r - s) + t * t)));
  else if (r < s)
    theta = 0;
  else
    theta = M_PI;
  // if (b > a) //original
  // {
  //   swap(a, b);
  //   theta += M_PI / 2;
  // }
  // if (t > 0.001)
  //   theta = std::atan(t / (s - r - std::sqrt((r - s) * (r - s) + t * t)));
  // else if (r - s < 0.001)
  //   theta = 0;
  // else
  //   theta = M_PI / 2;

  // ellipse = Ellipse{a, b, cx, cy, theta}; //original
  ellipse = Ellipse{cx, cy, theta, velocity, sigma_h, sigma_s, sigma_r, orient_rad}; //zbedit2
  return true;
}

Obstacle down_sample_points(const Obstacle &pts, int &x_min, int &y_min, int &x_max, int &y_max)
{
  for (const auto &pt : pts)
  {
    if (pt(0) < x_min)
      x_min = pt(0);
    if (pt(1) < y_min)
      y_min = pt(1);
    if (pt(0) > x_max)
      x_max = pt(0);
    if (pt(1) > y_max)
      y_max = pt(1);
  }

  int x_up = x_max - 0.1 * (x_max - x_min);
  x_up = x_up > x_min ? x_up : x_max;
  int x_down = x_min + 0.1 * (x_max - x_min);
  x_down = x_down < x_max ? x_down : x_min;
  int y_up = y_max - 0.1 * (y_max - y_min);
  y_up = y_up > y_min ? y_up : y_max;
  int y_down = y_min + 0.1 * (y_max - y_min);
  y_down = y_down < y_max ? y_down : y_min;

  if (pts.size() > 100)
  {
    Obstacle ds_pts;
    for (const auto &pt : pts)
    {
      if (pt(0) >= x_down && pt(0) <= x_up && pt(1) >= y_down && pt(1) <= y_up)
        continue;

      ds_pts.emplace_back(pt);
    }
    return ds_pts;
  }
  return pts;
}

vector<Ellipse> get_ellipse_array(std::vector<Obstacle> &seg_obs, const grid_map::GridMap &localmap_obs) //original
// vector<Ellipse> get_ellipse_array(std::vector<Obstacle> &seg_obs, const grid_map::GridMap &localmap_obs, std::vector<Obstacle> &prev_obs) //zbedit2
{
  std::vector<Ellipse> result;

  for (const auto &obs : seg_obs)
  {
    if (obs.size() > 500)
      continue;
    int x_min = localmap_x_size * _inv_resolution, y_min = localmap_x_size * _inv_resolution, x_max = 0, y_max = 0;
    Obstacle ds_pts = down_sample_points(obs, x_min, y_min, x_max, y_max);
    if (x_max - x_min > 3 * _inv_resolution || y_max - y_min > 3 * _inv_resolution)
      continue;

    vector<Point> pts;
    for (const auto &pt : ds_pts)
    {
      if (pt(0) >= localmap_x_size / resolution || pt(1) >= localmap_x_size / resolution)
        continue;

      Vector3d pos;
      localmap_obs.getPosition3("elevation", pt, pos);
      pts.emplace_back(pos(0), pos(1));
    }

    Ellipse ellipse;
    if (!get_min_area_ellipse(pts, ellipse))
    {
      Eigen::Array2i grid_center((x_max + x_min) / 2, (y_min + y_max) / 2);

      float radius = sqrt((x_max - x_min) * (x_max - x_min) + (y_max - y_min) * (y_max - y_min)) * resolution / 2;
      Vector3d pos;
      localmap_obs.getPosition3("gradient_map", grid_center, pos);
      ellipse.cx = pos(0);
      ellipse.cy = pos(1);
      // ellipse.semimajor = max(radius, 2 * resolution); //original
      // ellipse.semiminor = resolution; //original
      
      //zbedit2: Updated to reflect gaussian function    
      //zbedit2: Don't need to create special case for when vel=0, sigma_h already accounts for it with std::max 
      // ellipse.velocity = 0;
      // ellipse.orient_rad = 0; 
      // double dt = 0;
      // for (const auto &prev : prev_obs) {
      //   if (obs.label == prev.label) { //use label to ensure we are looking at position of the same obstacle
      //     dt = obs.time_stamp.toSec() - prev.time_stamp.toSec(); //http://wiki.ros.org/roscpp/Overview/Time
      //     ellipse.velocity = sqrt(pow(obs.x - prev.x, 2) + pow(obs.y - prev.y, 2))/dt;
      //     // ellipse.orient_rad = std::atan2(obs.y,obs.x) - std::atan2(prev.y,prev.x);
      //     ellipse.orient_rad = std::atan2((obs.y-prev.y),(obs.x-prev.x)); //zbtask: which calculations of orientation is correct?
      //     break; //found velocity so no point in continuing the for loop
      //   }
      // }
      // ellipse.sigma_h = std::max(2 * ellipse.velocity, 0.5); //zbedit2
      // ellipse.sigma_s = (2.0/3.0) * ellipse.sigma_h; //zbedit2
      // ellipse.sigma_r = 0.5 * ellipse.sigma_h; //zbedit2
      ellipse.theta = (x_max - x_min == 0) ? M_PI / 2 : (std::atan((y_max - y_min) / (x_max - x_min)) + M_PI / 2);
    }

    // if (ellipse.semimajor <= 20 && ellipse.semiminor <= 20) //original //?? what should the confidence condition be? Or should we push all results?
    //   result.push_back(ellipse);
    result.push_back(ellipse); //zbedit2
  }
  return result;
}

void ab_variance_calculation(vector<Ellipse> &input_vector)
{
  static vector<boost::circular_buffer<Eigen::Vector2d>> list;
  for (auto &input : input_vector)
  {
    int label = input.label;
    int current_size = list.size();
    if (current_size < label)
    {
      list.resize(label);
      for (size_t i = current_size; i < label; i++)
        list[i].set_capacity(10);
    }
    // list[label - 1].push_back(Eigen::Vector2d(input.semimajor, input.semiminor)); //original
    size_t list_size = list[label - 1].size();
    MatrixXd vac_mat(2, list_size);
    for (size_t i = 0; i < list_size; i++)
      vac_mat.col(i) = list[label - 1][i];
    MatrixXd zeroMeanMat = vac_mat.colwise() - vac_mat.rowwise().mean();
    MatrixXd covMat = (zeroMeanMat * zeroMeanMat.transpose()) / list_size;
    double variance = pow(covMat.trace(), 1.3);
    if (list_size < 3)
      variance += 0.0005 * (3 - list_size);
    input.variance = variance;
  }
}

void asym_guass_calculation(vector<Ellipse> &input_curr, vector<Ellipse> &input_prev)
{
  for (int i = 0; i < input_curr.size(); i++){
    input_curr[i].velocity = 0;
    input_curr[i].orient_rad = 0; 
    double dt = 0;
    for (const auto &prev : input_prev) {
      if (input_curr[i].label == prev.label) { //use label to ensure we are looking at position of the same obstacle
        dt = input_curr[i].time_stamp.toSec() - prev.time_stamp.toSec(); //http://wiki.ros.org/roscpp/Overview/Time
        input_curr[i].velocity = sqrt(pow(input_curr[i].cx - prev.cx, 2) + pow(input_curr[i].cy - prev.cy, 2))/dt;
        // ellipse.orient_rad = std::atan2(obs.y,obs.x) - std::atan2(prev.y,prev.x);
        input_curr[i].orient_rad = std::atan2((input_curr[i].cy-prev.cy),(input_curr[i].cx-prev.cx)); //zbtask: which calculations of orientation is correct?
        break; //found velocity so no point in continuing the for loop
      }
    }
    input_curr[i].sigma_h = std::max(2 * input_curr[i].velocity, 0.5); //zbedit2
    input_curr[i].sigma_s = (2.0/3.0) * input_curr[i].sigma_h; //zbedit2
    input_curr[i].sigma_r = 0.5 * input_curr[i].sigma_h; //zbedit2
  }
 
}

void visEllipse(const std::vector<Ellipse> &obs_ellipses, ros::Publisher &_Ellipses_vis_pub)
{
  // ellipse
  visualization_msgs::MarkerArray ellipse_vis;

  visualization_msgs::Marker shape_vis;
  shape_vis.header.frame_id = "map";
  shape_vis.header.stamp = ros::Time::now();
  shape_vis.ns = "ellipse";
  // shape_vis.type = visualization_msgs::Marker::CYLINDER; //original
  shape_vis.type = visualization_msgs::Marker::LINE_STRIP; //SPHERE; //zbedit2
  shape_vis.action = visualization_msgs::Marker::ADD;
  shape_vis.lifetime = ros::Duration(0.15);

  // number
  visualization_msgs::Marker text_vis;
  text_vis.header.frame_id = "map";
  text_vis.header.stamp = ros::Time::now();
  text_vis.ns = "text";
  text_vis.action = visualization_msgs::Marker::ADD;
  text_vis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_vis.lifetime = ros::Duration(0.1);
  text_vis.scale.z = 0.3;
  text_vis.color.r = 1;
  text_vis.color.g = 1;
  text_vis.color.b = 1;
  text_vis.color.a = 1;

  for (size_t i = 0; i < obs_ellipses.size(); i++)
  {
    shape_vis.color.a = 0.9;
    if (obs_ellipses[i].variance < 0.0007)
    {
      shape_vis.color.r = 0;
      shape_vis.color.g = 0;
      shape_vis.color.b = 1;
    }
    else if (obs_ellipses[i].variance > 0.005)
    {
      shape_vis.color.r = 1;
      shape_vis.color.g = 0;
      shape_vis.color.b = 0;
    }
    else
    {
      float dt = (obs_ellipses[i].variance - 0.0007) / (0.005 - 0.0007);
      if (dt > 0.5)
      {
        shape_vis.color.r = 1;
        shape_vis.color.g = 1 - dt;
        shape_vis.color.b = 1 - dt;
      }
      else
      {
        shape_vis.color.r = dt;
        shape_vis.color.g = dt;
        shape_vis.color.b = 1;
      }
    }

    shape_vis.id = i;
    shape_vis.pose.orientation.x = 0.0;
    shape_vis.pose.orientation.y = 0.0;
    shape_vis.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
    shape_vis.pose.orientation.w = cos(obs_ellipses[i].theta / 2);
    shape_vis.pose.position.x = obs_ellipses[i].cx;
    shape_vis.pose.position.y = obs_ellipses[i].cy;
    shape_vis.pose.position.z = -0.0;
    // shape_vis.scale.x = 2 * obs_ellipses[i].semimajor; //original
    // shape_vis.scale.y = 2 * obs_ellipses[i].semiminor; //original
    // shape_vis.scale.z = 0.75; //original
    shape_vis.scale.x = obs_ellipses[i].sigma_s; //zbedit2
    shape_vis.scale.y = obs_ellipses[i].sigma_s; //zbedit2
    shape_vis.scale.z = 1.0; //zbedit2

    //zbedit2: assign sigma value based on orientation:
    //-----------------------------------------------------------------------------------
    double vel_ang = fmod(obs_ellipses[i].orient_rad + 2 * M_PI,  2 * M_PI); //zbtask: Is it 30+sig_h?
    double ray_ang = 0;
    double delta_ang = M_PI / 30;
    double cons = 0.354163;

    // zbedit2: (op1)
    while (ray_ang <= 2 * M_PI){
      int flag = int(ray_ang / (M_PI / 2));

      double prev_variance;
      double next_variance;

      if (flag == 0){
          prev_variance = obs_ellipses[i].sigma_h;
          next_variance = obs_ellipses[i].sigma_s;
      }
      else if( flag == 1){
          prev_variance = obs_ellipses[i].sigma_r;
          next_variance = obs_ellipses[i].sigma_s;
      }
      else if( flag == 2){
          prev_variance = obs_ellipses[i].sigma_r;
          next_variance = obs_ellipses[i].sigma_s;
      }
      else{
          prev_variance = obs_ellipses[i].sigma_h;
          next_variance = obs_ellipses[i].sigma_s;
      }

      double dist = sqrt(cons / ((cos(ray_ang) *cos(ray_ang) / (2 * prev_variance)) + (sin(ray_ang) *sin(ray_ang) / (2 * next_variance))));

      double total_angle = ray_ang + vel_ang;

      double x_offset = dist * cos(total_angle);
      double y_offset = dist * sin(total_angle);

      geometry_msgs::Point edge_point;

      edge_point.x = obs_ellipses[i].cx + x_offset;
      edge_point.y = obs_ellipses[i].cy + y_offset;
      edge_point.z = 0;
      
      shape_vis.points.push_back(edge_point);

      ray_ang += delta_ang;
    }
    
    // //zbedit2: (op2)
    // while (ray_ang <= 2 * M_PI){
    //   //zbtask: How do I give the 60 points to shape_vis? 
    //   //zbedit2: Depending on the orientation measured for the obstacle, the scale is based on where it lies on the unit circle

    //   curr_ang = fmod(ray_ang - obs_ellipses[i].orient_rad, 2*M_PI)

    //   geometry_msgs::Point edge_point;

    //   if (curr_ang <= M_PI/2){
    //     edge_point.x = cos(curr_ang) * sigma_h + obs_ellipses[i].cx; 
    //     edge_point.y = sin(curr_ang) * sigma_s + obs_ellipses[i].cy;
    //   }
    //   else if (curr_ang <= M_PI){
    //     edge_point.x = sin(curr_ang - M_PI/2) * sigma_r + obs_ellipses[i].cx; 
    //     edge_point.y = cos(curr_ang - M_PI/2) * sigma_s + obs_ellipses[i].cy;
    //   }
    //   else if (curr_ang <= 3*M_PI/2){
    //     edge_point.x = cos(curr_ang - M_PI) * sigma_r + obs_ellipses[i].cx; 
    //     edge_point.y = sin(curr_ang - M_PI) * sigma_s + obs_ellipses[i].cy;
    //   }
    //   else {
    //     edge_point.x = sin(curr_ang - 3*M_PI/2) * sigma_h + obs_ellipses[i].cx; 
    //     edge_point.y = cos(curr_ang - 3*M_PI/2) * sigma_s + obs_ellipses[i].cy;
    //   }

    //   mk.points.push_back(edge_point);

    //   ray_ang += delta_ang;
    // }
    // mk_list.markers.push_back(mk);
    //-----------------------------------------------------------------------------------

    text_vis.id = obs_ellipses[i].label;

    text_vis.text = to_string(obs_ellipses[i].label);
    text_vis.pose.position.x = obs_ellipses[i].cx - 0.3;
    text_vis.pose.position.y = obs_ellipses[i].cy;
    text_vis.pose.position.z = 0.2;

    // publish
    ellipse_vis.markers.push_back(shape_vis);
    ellipse_vis.markers.push_back(text_vis);
  }

  _Ellipses_vis_pub.publish(ellipse_vis);
}
