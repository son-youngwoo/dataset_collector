#include <ros/ros.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Events.hh>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <sensor_msgs/Imu.h>
#include <dataset_collector/dataset_rec_model.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <random>
#include <iostream>
#include <cmath>

#include <fstream>
#include <Eigen/Dense>

#ifndef pi
#define pi 3.14159265359
#endif

#ifndef DEG2RAD
#define DEG2RAD (pi / 180.0)
#endif

using namespace std;
using namespace Eigen;

std::string robot_namespace;

/////////////////

double roll_limit = 0.1;
double pitch_limit = 0.1;
double height_limit = 0.3;

// respawn_rec_model.cpp
// int step = 1;

// respawn_rec_model_base.cpp
int step = 2;
int id = 1;

int map_x = 0;
int map_y = 0;

int cnt = 0;
int cnt1 = 0;
int cnt2 = 0;
int cnt_respawn = 0;

int flag_turn1 = 0;
int flag_turn2 = 0;
int flag_init = 1;
int flag_respawn = 0;

double left_yaw_max1 = 0;
double left_yaw_max2 = 0;
double right_yaw_max1 = 0;
double right_yaw_max2 = 0;

double left_yaw_max1_local = 0;
double left_yaw_max2_local = 0;
double right_yaw_max1_local = 0;
double right_yaw_max2_local = 0;

double left_yaw_max1_deg = 0;
double left_yaw_max2_deg = 0;
double right_yaw_max1_deg = 0;
double right_yaw_max2_deg = 0;

double angle1 = 0;
double angle2 = 0;

double yaw_vel = 0;
double base_yaw = 0;

double yaw_init = 0;
double rand_x_init = 0;
double rand_y_init = 0;

double cur_z_world = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

double kp = 50;
double kd = 70;
double torque = 0;

//////////////////

ros::Subscriber sub_bodypose;
ros::Subscriber sub_imu;

ros::Publisher pub_torque;
ros::Publisher pub_dataset_rec_model;

ros::ServiceClient SetModelStateClient;

dataset_collector::dataset_rec_model dataset_rec_model;

ofstream outfile;

void msgCallbackIMU(const sensor_msgs::Imu::ConstPtr& msg) // son
{
    // Convert the orientation quaternion to a rotation matrix
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw); // roll, pitch, yaw of body from world frame

    yaw_vel = msg->angular_velocity.z; // yaw velocity of body from world frame
}

void msgCallbackBodyPose(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    cur_z_world = msg->data[2]; // z position of base from world frame
    base_yaw = msg->data[5]; // yaw angle of base from world frame
}

void ROSInit(ros::NodeHandle &_nh)
{

    sub_bodypose = _nh.subscribe("BodyPose_sim", 10, msgCallbackBodyPose);
    sub_imu = _nh.subscribe("IMU_sim", 10, msgCallbackIMU);

    pub_dataset_rec_model = _nh.advertise<dataset_collector::dataset_rec_model>("dataset_rec_model", 100);
    pub_torque = _nh.advertise<std_msgs::Float32>("Torque_sim", 100);

    SetModelStateClient = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

void Rotate(float _torque)
{
    // _torque > 0 -> rotate right
    // _torque < 0 -> rotate left

    std_msgs::Float32 torque;
    torque.data = _torque;
    pub_torque.publish(torque);
}

void getRandomPos(double &x, double &y)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> dis_init_x(-9 + map_x, 9 + map_x);
    std::uniform_real_distribution<double> dis_init_y(-9 + map_y, 9 + map_y);

    x = dis_init_x(gen);
    y = dis_init_y(gen);
}

void Respawn(double _rand_x_init, double _rand_y_init, double _yaw_init)
{
    _yaw_init = _yaw_init * DEG2RAD;

    if (_rand_x_init == 0 && _rand_y_init == -0.5 && _yaw_init == 0) {
        
        _rand_x_init = _rand_x_init + map_x;
        _rand_y_init = _rand_y_init + map_y;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, _yaw_init);

    gazebo_msgs::ModelState modelState;
    modelState.model_name = robot_namespace;
    modelState.pose.position.x = _rand_x_init; 
    modelState.pose.position.y = _rand_y_init;
    modelState.pose.position.z = 1.0;
    modelState.pose.orientation.x = q.x(); 
    modelState.pose.orientation.y = q.y();
    modelState.pose.orientation.z = q.z();
    modelState.pose.orientation.w = q.w();

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = modelState;
    SetModelStateClient.call(srv);
}