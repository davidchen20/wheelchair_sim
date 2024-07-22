#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

#include "kalman.h"

using namespace std;
using namespace Eigen;

ros::Subscriber _obs_sub;
ros::Publisher _obs_pub, _obs_vis_pub;

const size_t obs_kf_buffer_size = 100;

struct Ellipse
{
  // double semimajor;  // Length of semi-major axis //original
  // double semiminor;  // Length of semi-minor axis //original
  double cx;         // x-coordinate of center
  double cy;         // y-coordinate of center
  double theta;      // Rotation angle
  // double velocity; //zbedit2
  double sigma_h; //zbedit2
  double sigma_s; //zbedit2
  double sigma_r; //zbedit2
};

struct obs_param
{
  float x = 0.0;
  float y = 0.0;
  // float a = 0.0; //original
  // float b = 0.0; //original
  float theta = 0.0;
  float mea_cov;
  // float velocity = 0.0; //zbedit2
  float sigma_h = 0.0; //zbedit2
  float sigma_s = 0.0; //zbedit2
  float sigma_r = 0.0; //zbedit2
};

class obs_kf
{
public:
  int N;
  float T;

  Kalman ka;
  std::vector<obs_param> param_list;
  std::vector<obs_param> pred_list;
  Eigen::VectorXd x;
  Eigen::MatrixXd P0;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd H;
  Eigen::VectorXd z0;
  Eigen::VectorXd u_v;

  void obs_predict();
  obs_kf();
} _obs_kf[obs_kf_buffer_size];

obs_kf::obs_kf() : N(25)
{
  ka.m_StateSize = 9; //original
  // ka.m_MeaSize = 5; //original
  ka.m_USize = 9; //original
  // ka.m_StateSize = 11; //zbedit2 //reverting back to original value of 9 because of KF paramaters
  ka.m_MeaSize = 6; //zbedit2
  // ka.m_USize = 11; //zbedit2

  T = 0.1;

  x.resize(9); //original
  // x.resize(11); //zbedit2
  x.setZero();

  float P0_cov = 0.025;

  P0.resize(9, 9); //original
  // P0.resize(11, 11); //zbedit2
  P0.setIdentity();
  P0 *= P0_cov;

  float Q_cov = 0.00008;

  Q.resize(9, 9); //original]
  // Q.resize(11, 11); //zbedit2
  Q.setIdentity();
  Q *= Q_cov;

  float R_cov = 0.0008;  //仿真 0.00005  实物  0.000003

  // R.resize(5, 5); //original
  R.resize(6, 6); //zbedit2
  R.setZero();

  // R(0, 0) = R_cov; //original
  // R(1, 1) = R_cov; //original
  // R(2, 2) = 10 * R_cov; //original
  // R(3, 3) = 10 * R_cov; //original
  // R(4, 4) = 0.00000001 * R_cov; //original
  R(0, 0) = R_cov; //x zbedit2
  R(1, 1) = R_cov; //y zbedit2
  // R(2, 2) = 0.01 * R_cov; //velocity zbedit2
  R(2, 2) = 0.0001 * R_cov; //sigma_h zbedit2
  R(3, 3) = 0.0001 * R_cov; //sigma_s zbedit2
  R(4, 4) = 0.0001 * R_cov; //sigma_r zbedit2
  R(5, 5) = 0.00000001 * R_cov; //zbedit2

  A.resize(9, 9); //original
  // A.resize(11, 11); //zbedit2
  A.setIdentity();
  // A(0, 5) = A(1, 6) = T; //original
  A(0, 6) = A(1, 7) = T; //zbedit2

  //     A<<1,0,0,0,0,T,0,0.5*T*T,0,
  //             0,1,0,0,0,0,T,0,0.5*T*T,
  //             0,0,1,0,0,0,0,0,0,
  //             0,0,0,1,0,0,0,0,0,
  //             0,0,0,0,1,0,0,0,0,
  //             0,0,0,0,0,1,0,T,0,
  //             0,0,0,0,0,0,1,0,T,
  //             0,0,0,0,0,0,0,1,0,
  //             0,0,0,0,0,0,0,0,1;

  B.resize(9, 9); //original
  // B.resize(11, 11);
  B.setZero();

  // H.resize(5, 9); //original
  // H.resize(6, 11); //zbedit2
  H.resize(6, 9); //zbedit2
  H.setIdentity();

  // z0.resize(5); //original
  z0.resize(6); //zbedit2
  z0.setZero();

  u_v.resize(9); //original
  // u_v.resize(11); //zbedit2
  u_v.setZero();

  ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
}

void obs_kf::obs_predict()
{
  int num = param_list.size();
  if (num == 0)
    return;

  if (num == 1)
  {
    // ka.m_x << param_list[0].x, param_list[0].y, param_list[0].a, param_list[0].b, param_list[0].theta, 0, 0, 0, 0; //original
    ka.m_x << param_list[0].x, param_list[0].y, param_list[0].sigma_h, param_list[0].sigma_s, param_list[0].sigma_r, param_list[0].theta, 0, 0, 0; //zbedit2
    ka.m_P = P0;
  }
  else
  {
    float _mea_cov_min = 0.005;
    float _mea_cov_max = 0.05;
    float _k_cov_min = 1;
    float _k_cov_max = 3000;
    float _mea_cov = min(max(param_list.back().mea_cov, _mea_cov_min), _mea_cov_max);
    float _k = log10(_mea_cov / _mea_cov_min) / log10(_mea_cov_max / _mea_cov_min);
    float _k_cov = pow(_k_cov_max, _k) * pow(_k_cov_min, 1 - _k);

    ka.m_R(0, 0) = _k_cov * R(0, 0);
    ka.m_R(1, 1) = _k_cov * R(1, 1);

    // VectorXd z(5); //original
    VectorXd z(6); //zbedit2
    // z << param_list.back().x, param_list.back().y, param_list.back().a, param_list.back().b, param_list.back().theta; //original
    z << param_list.back().x, param_list.back().y, param_list.back().sigma_h, param_list.back().sigma_s, param_list.back().sigma_r, param_list.back().theta; //zbedit2

    ka.Predict_State();
    ka.Predict_Cov();
    ka.Mea_Resd(z);
    ka.Cal_Gain();
    ka.Update_State();
    ka.Update_Cov();

    Kalman ka_tmp = ka;

    pred_list.clear();
    for (int i = 0; i < N; i++)
    {
      ka_tmp.Predict_State();
      ka_tmp.Predict_Cov();
      obs_param pred;
      float x_conf = 0.95 * sqrt(ka_tmp.m_P(0, 0));
      float y_conf = 0.95 * sqrt(ka_tmp.m_P(1, 1));
      // float a_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2)); //original
      // float b_conf = 0.95 * sqrt(ka_tmp.m_P(3, 3)); //original
      // float velocity_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2)); //zbedit2
      float sigma_h_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2)); //zbedit2
      float sigma_s_conf = 0.95 * sqrt(ka_tmp.m_P(3, 3)); //zbedit2
      float sigma_r_conf = 0.95 * sqrt(ka_tmp.m_P(4, 4)); //zbedit2
      float pos_conf = max(x_conf, y_conf);
      // float ab_conf = max(a_conf, b_conf); //original

      pred.x = ka_tmp.m_x(0);
      pred.y = ka_tmp.m_x(1);
      // pred.a = ka_tmp.m_x(2) + pos_conf + ab_conf; //original
      // pred.b = ka_tmp.m_x(3) + pos_conf + ab_conf; //original
      // pred.velocity = ka_tmp.m_x(2); //zbedit2
      pred.sigma_h = ka_tmp.m_x(2); //zbedit2
      pred.sigma_s = ka_tmp.m_x(3); //zbedit2
      pred.sigma_r = ka_tmp.m_x(4); //zbedit2
      // pred.theta = ka.m_x(4); //original
      pred.theta = ka.m_x(5); //zbedit2

      pred_list.push_back(pred);
    }
  }
}

void visEllipse(const vector<Ellipse>& obs_ellipses)
{
  visualization_msgs::MarkerArray ellipse_vis;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.ns = "ellipse";
  // mk.type = visualization_msgs::Marker::CYLINDER; //original
  mk.type = visualization_msgs::Marker::LINE_STRIP; //SPHERE; //zbedit2
  mk.action = visualization_msgs::Marker::ADD;
  mk.lifetime = ros::Duration(0.5);

  for (size_t i = 0; i < obs_ellipses.size(); i++)
  {
    mk.color.a = 0.4;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0;

    mk.id = i;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
    mk.pose.orientation.w = cos(obs_ellipses[i].theta / 2);

    mk.pose.position.x = obs_ellipses[i].cx;
    mk.pose.position.y = obs_ellipses[i].cy;
    mk.pose.position.z = -0.3;

    // mk.scale.x = 2 * obs_ellipses[i].semimajor; //original
    // mk.scale.y = 2 * obs_ellipses[i].semiminor; //original
    // mk.scale.z = 0.1; //original
    mk.scale.x = obs_ellipses[i].sigma_s; //zbedit2
    mk.scale.y = obs_ellipses[i].sigma_s; //zbedit2
    mk.scale.z = 1.0; //zbedit2

    ellipse_vis.markers.push_back(mk);
  }

  _obs_vis_pub.publish(ellipse_vis);
}

void curve_fitting(obs_kf& obs, std_msgs::Float32MultiArray& obs_pub, vector<Ellipse>& ellipses_array)
{
  if (obs.param_list.empty())
    return;

  obs.obs_predict();

  for (int i = 0; i < obs.pred_list.size(); i++)
  {
    Ellipse ellipse;
    ellipse.cx = obs.pred_list[i].x;
    ellipse.cy = obs.pred_list[i].y;
    // ellipse.semimajor = obs.pred_list[i].a; //original
    // ellipse.semiminor = obs.pred_list[i].b; //original
    // ellipse.velocity = obs.pred_list[i].velocity; //zbedit2
    ellipse.sigma_h = obs.pred_list[i].sigma_h; //zbedit2
    ellipse.sigma_s = obs.pred_list[i].sigma_s; //zbedit2
    ellipse.sigma_r = obs.pred_list[i].sigma_r; //zbedit2
    ellipse.theta = obs.pred_list[i].theta;
    ellipses_array.push_back(ellipse);

    obs_pub.data.push_back(obs.pred_list[i].x);
    obs_pub.data.push_back(obs.pred_list[i].y);
    // obs_pub.data.push_back(obs.pred_list[i].a); //original
    // obs_pub.data.push_back(obs.pred_list[i].b); //original
    // obs_pub.data.push_back(obs.pred_list[i].velocity); //zbedit2
    obs_pub.data.push_back(obs.pred_list[i].sigma_h); //zbedit2
    obs_pub.data.push_back(obs.pred_list[i].sigma_s); //zbedit2
    obs_pub.data.push_back(obs.pred_list[i].sigma_r); //zbedit2
    obs_pub.data.push_back(obs.pred_list[i].theta);
  }
}

void obscb(const std_msgs::Float32MultiArray::ConstPtr& msg) //original
// void obscb(const std_msgs::Float32MultiArray::ConstPtr& msg, obs_param &obs_sub_prev) //zbedit2
{
  ROS_INFO("[node] receive the obs track");
  if (msg->data.empty())
    return;

  int num = msg->data.size() / 7;

  std_msgs::Float32MultiArray obs_pub;
  vector<Ellipse> ellipses_array;
  for (int i = 0; i < num; i++)
  {
    int flag = msg->data[7 * i + 5];

    obs_param _obs_tmp;
    _obs_tmp.x = msg->data[7 * i];
    _obs_tmp.y = msg->data[7 * i + 1];
    // _obs_tmp.a = msg->data[7 * i + 2]; //original
    // _obs_tmp.b = msg->data[7 * i + 3]; //original
    // _obs_tmp.theta = msg->data[7 * i + 4]; //original
    // _obs_tmp.mea_cov = msg->data[7 * i + 6]; //original
    _obs_tmp.theta = msg->data[7 * i + 2]; //zbedit2
    _obs_tmp.mea_cov = msg->data[7 * i + 4]; //zbedit2
    _obs_tmp.sigma_h = msg->data[7 * i + 6]; //zbedit2
    _obs_tmp.sigma_s = msg->data[7 * i + 7]; //zbedit2
    _obs_tmp.sigma_r = msg->data[7 * i + 8]; //zbedit2

    // zbedit2: Don't need to do this b/c I am using the published data instead
    // _obs_tmp.velocity = 0;
    // double dt = 0;
    // for (const auto &prev : obs_sub_prev) {
    //   if (_obs_tmp.label == obs_sub_prev.label) { //use label to ensure we are looking at position of the same obstacle
    //     dt = _obs_tmp.time.toSec() - obs_sub_prev.time.toSec(); 
    //     _obs_tmp.velocity = sqrt(pow(_obs_tmp.x - obs_sub_prev.x, 2) + pow(_obs_tmp.y - obs_sub_prev.y, 2))/dt;
    //     break; //found velocity so no point in continuing the for loop
    //   }
    // }
    // _obs_tmp.sigma_h = std::max(2 * _obs_tmp.velocity, 0.5); //zbedit2
    // _obs_tmp.sigma_s = (2.0/3.0) * _obs_tmp.sigma_h; //zbedit2
    // _obs_tmp.sigma_r = 0.5 * _obs_tmp.sigma_h; //zbedit2

    _obs_kf[flag].param_list.push_back(_obs_tmp);

    curve_fitting(_obs_kf[flag], obs_pub, ellipses_array);
  }

  _obs_pub.publish(obs_pub);
  visEllipse(ellipses_array);
  // obs_sub_prev = obs_sub; //zbedit2 //??can I do this??
  // obs_sub_prev.time = obs_sub.time; //zbedit2
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obs_param_node");
  ros::NodeHandle n;
  _obs_sub = n.subscribe("/local_map_pub/for_obs_track", 1, obscb); //original
  // No need to initialize obs_sub_prev here anymore, instead using published data
  // obs_param obs_sub_prev; //zbedit2
  // obs_sub_prev.time = ros::Time::now(); //zbedit2
  // _obs_sub = n.subscribe("/local_map_pub/for_obs_track", 1, boost::bind(obscb, _1, obs_sub_prev)); //zbedit2
  _obs_pub = n.advertise<std_msgs::Float32MultiArray>("/obs_predict_pub", 1);
  _obs_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/obs_predict_vis_pub", 1);
  ros::spin();
  return 0;
}
