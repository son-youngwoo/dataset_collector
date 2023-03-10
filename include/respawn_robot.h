#include <ros/ros.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <dataset_collector/dataset.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <random>
#include <iostream>
#include <cmath>

int cnt1 = 0;
int step = 2;
bool get_s_or_f = 0;
bool overturn = 0;
double x = 0;
double y = 0;
double z = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;
double roll_limit = 0.7;
double pitch_limit = 0.7;
int data_id = 1;
bool s_or_f;
int cnt_ready = 0;
int cnt_s_or_f = 0;
int cnt_path = 0;
int cnt_vel = 0;
int cnt_data = 0;
int cnt_respawn = 0;

double yaw_init = 0;
double yaw_target = 0;
double _yaw_target = 0;
double __yaw_target = 0;
double yaw_target_dis = 0;
double rand_x_tar = 0;
double rand_y_tar = 0;
double x_init = 0;
double y_init = 0;
int num_div = 0;
double yaw_target_deg = 0;
double pos_x = 0;
double pos_y = 0;

double d = 0;
double R_success = 0;
double xvel_target = 0;

double min_radius = 0.8;
double max_radius = 1.0;

void Reset();

using namespace std;

ros::Subscriber sub_bodypose;
ros::Subscriber sub_elevationmap;

ros::Publisher pub_path;
ros::Publisher pub_zerotorqueflag;
ros::Publisher pub_controlinput;
ros::Publisher pub_dataset;
ros::Publisher pub_xvel;
ros::Publisher pub_vel;

ros::ServiceClient client;

dataset_collector::dataset dataset;
grid_map_msgs::GridMap elevation_map_raw_copy;
grid_map_msgs::GridMap elevation_map_raw;
std_msgs::Int8 controlinput;

void msgCallbackBodyPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    x = msg->data[0];
    y = msg->data[1];
    z = msg->data[2];
    roll = msg->data[3];
    pitch = msg->data[4];
    yaw = msg->data[5];
}

void msgCallbackElevationMap(const grid_map_msgs::GridMap& msg)
{
    elevation_map_raw_copy = msg;
}

void ROSInit(ros::NodeHandle& _nh)
{
    sub_bodypose = _nh.subscribe("/aidin81/BodyPose_sim", 10, msgCallbackBodyPose);
    sub_elevationmap = _nh.subscribe("/aidin81/elevation_mapping/elevation_map_raw", 10, msgCallbackElevationMap);

    pub_path = _nh.advertise<nav_msgs::Path>("/aidin81/Path", 100);
    pub_zerotorqueflag = _nh.advertise<std_msgs::Bool>("/aidin81/ZeroTorqueFlag", 100);
    pub_controlinput = _nh.advertise<std_msgs::Int8>("/aidin81/ControlInput", 100);
    pub_dataset = _nh.advertise<dataset_collector::dataset>("/aidin81/dataset", 100);
    pub_xvel = _nh.advertise<std_msgs::Float32>("/aidin81/xvel_target", 100);
    pub_vel = _nh.advertise<std_msgs::Float32MultiArray>("/aidin81/vel_target", 100);

    client = _nh.serviceClient<std_srvs::Empty>("/aidin81/elevation_mapping/clear_map");
}


void PubZeroTorqueFlag(bool _zerotorqueflag){
    std_msgs::Bool zerotorqueflag;
    zerotorqueflag.data = _zerotorqueflag;
    pub_zerotorqueflag.publish(zerotorqueflag);
}

void InitializeSensor() {
    // std::cout << "1. initialize imu roll, pitch & encoder" << std::endl;
    controlinput.data = 1;
    pub_controlinput.publish(controlinput);
}

void InitializeYaw() {
    // std::cout << "2. initialize imu yaw" << std::endl;
    controlinput.data = 2;
    pub_controlinput.publish(controlinput);
}

void StartMode() {
    // std::cout << "3. crawl"   << std::endl;  
    controlinput.data = 9;
    pub_controlinput.publish(controlinput);
}

void CrawlMode() {
    // std::cout << "3. crawl"   << std::endl;  
    controlinput.data = 3;
    pub_controlinput.publish(controlinput);
}

void StandMode() {
    // std::cout << "4. stand"  << std::endl;
    controlinput.data = 4;
    pub_controlinput.publish(controlinput);
}

void MPCMode() {
    // std::cout << "5. mpc" << std::endl;
    controlinput.data = 5;
    pub_controlinput.publish(controlinput); 
}

void ResetElevationMap() {
    // std::cout << "elevation map reset" << std::endl;
    std_srvs::Empty srv;
    if (client.call(srv)) {
        std::cout << "  > elevation map cleared" << std::endl;
    } else {
        //ROS_ERROR("Failed to call service /aidin81/elevation_mapping/clear_map");
    }
}

void PublishZeroVelocity() {
    std_msgs::Float32 xvel_target;
    xvel_target.data = 0;
    pub_xvel.publish(xvel_target);
}

void PublishVelocity() {
    std_msgs::Float32 xvel_target;
    xvel_target.data = 0.1;
    pub_xvel.publish(xvel_target);
}

void PublishZeroPath() {
    nav_msgs::Path path;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    path.poses.resize(2);  // allocate memory for the poses array

    path.poses[0].header.frame_id = "world";
    path.poses[0].header.stamp = ros::Time::now();
    path.poses[0].pose.position.x = 0;
    path.poses[0].pose.position.y = 0;
    path.poses[0].pose.orientation.x = 0;
    path.poses[0].pose.orientation.y = 0;
    path.poses[0].pose.orientation.z = 0;
    path.poses[0].pose.orientation.w = 1;

    path.poses[1].header.frame_id = "world";
    path.poses[1].header.stamp = ros::Time::now();
    path.poses[1].pose.position.x = 0;
    path.poses[1].pose.position.y = 0;
    path.poses[1].pose.orientation.x = 0;
    path.poses[1].pose.orientation.y = 0;
    path.poses[1].pose.orientation.z = 0;
    path.poses[1].pose.orientation.w = 1;

    pub_path.publish(path);
}

void Respawn() {
    //std::cout << "respawn ..." << std::endl;        

    // create a random number generator engine
    std::random_device rd_init;
    std::mt19937 gen_init(rd_init());

    // create a distribution that generates random double numbers in [0.0, 1.0)
    std::uniform_real_distribution<double> dis_init(-5.0, 5.0);

    double rand_x_init = dis_init(gen_init);
    double rand_y_init = dis_init(gen_init);

    gazebo_msgs::ModelState modelState;
    modelState.model_name = "aidin81";  // Replace with your robot's name
    modelState.pose.position.x = rand_x_init;   // Replace with your robot's starting position
    modelState.pose.position.y = rand_y_init;
    modelState.pose.position.z = 0.5;
    modelState.pose.orientation.x = 0.0;  // Replace with your robot's starting orientation
    modelState.pose.orientation.y = 0.0;
    modelState.pose.orientation.z = 0.0;
    modelState.pose.orientation.w = 1;
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = modelState;
    if (client.call(srv))
    {
        // ROS_INFO("Robot respawned successfully");
        //std::cout << "respawn x: " << rand_x_init << " y: " << rand_y_init << std::endl;        
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
    }
}

void PublishPath(int _cnt_path){
    if (_cnt_path == 1) {
        //std::cout << "publish target position" << std::endl;        

        x_init = x;
        y_init = y;
        
        std::cout << "global_initial_x: " << x_init << std::endl;        // test
        std::cout << "global_initial_y: " << y_init << std::endl;        // test

        // Define the random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_angle(-M_PI, M_PI);
        std::uniform_real_distribution<> dis_radius(min_radius, max_radius);
        
        // Generate a random angle and distance
        double angle = dis_angle(gen);
        double radius = dis_radius(gen);
        
        // rand_x_tar = x_init + radius * std::cos(angle + yaw_init); // world base
        // rand_y_tar = y_init + radius * std::sin(angle + yaw_init); // world base
        // ROS_ERROR("local 기준 angle :%f", (angle)/M_PI*180);

        // std::cout << "local_initial_x: " << rand_x_tar - x_init << std::endl;       // test 
        // std::cout << "local_initial_y: " << rand_y_tar - y_init << std::endl;       // test
        
        // // pos_x = cos(-yaw_init)*(rand_x_tar - x)-sin(-yaw_init)*(rand_y_tar - y);
        // // pos_y = sin(-yaw_init)*(rand_x_tar - x)+cos(-yaw_init)*(rand_y_tar - y);
        // ROS_ERROR("yaw_init : %f", yaw_init);
        // ROS_ERROR("rand_x_tar : %f", rand_x_tar);
        // ROS_ERROR("rand_x_tar : %f", rand_y_tar);


        // // yaw_target = atan2(rand_y_tar - y, rand_x_tar - x); // theta based world frame
        // yaw_target = angle; // theta based world frame
        // ROS_ERROR("yaw_target : %f", yaw_target);

        // yaw_target_deg = abs(yaw_target/M_PI*180);

        // num_div = yaw_target_deg / 3;
        
        // yaw_target_dis = yaw_target / num_div; 

        // Calculate the x and y coordinates
        rand_x_tar = x_init + radius * std::cos(angle); // world base
        rand_y_tar = y_init + radius * std::sin(angle); // world base
        ROS_ERROR("angle :%f", angle/M_PI*180);


        std::cout << "local_initial_x: " << rand_x_tar - x_init << std::endl;       // test 
        std::cout << "local_initial_y: " << rand_y_tar - y_init << std::endl;       // test
        
        pos_x = cos(-yaw_init)*(rand_x_tar - x)-sin(-yaw_init)*(rand_y_tar - y);
        pos_y = sin(-yaw_init)*(rand_x_tar - x)+cos(-yaw_init)*(rand_y_tar - y);
        ROS_ERROR("yaw_init : %f", yaw_init);
        ROS_ERROR("pos_x : %f", pos_x);
        ROS_ERROR("pos_y : %f", pos_y);


        yaw_target = atan2(rand_y_tar - y, rand_x_tar - x); // theta based world frame = angle 
        // yaw_target = atan2(pos_y, pos_x); // theta based world frame = angle + yaw_init
        // yaw_target = angle + yaw_init; // theta based world frame

        ROS_ERROR("yaw_target : %f", yaw_target);

        yaw_target_deg = abs(yaw_target/M_PI*180);

        num_div = yaw_target_deg / 3;
        
        yaw_target_dis = yaw_target / num_div;

    }

    if(_cnt_path < num_div + 1) {
        _yaw_target = yaw_target_dis*_cnt_path;
    }
    else {
        _yaw_target = yaw_target_dis*num_div;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, _yaw_target);

    tf2::Quaternion q_init;
    q_init.setRPY(0, 0, yaw_init);

    nav_msgs::Path path;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.resize(2);  // allocate memory for the poses array
    path.poses[0].header.frame_id = "world";
    path.poses[0].header.stamp = ros::Time::now();
    path.poses[0].pose.position.x = x_init;
    path.poses[0].pose.position.y = y_init;
    path.poses[0].pose.orientation.x = q_init.x();
    path.poses[0].pose.orientation.y = q_init.y();
    path.poses[0].pose.orientation.z = q_init.z();
    path.poses[0].pose.orientation.w = q_init.w();

    path.poses[1].header.frame_id = "world";
    path.poses[1].header.stamp = ros::Time::now();
    path.poses[1].pose.position.x = rand_x_tar;
    path.poses[1].pose.position.y = rand_y_tar;
    path.poses[1].pose.orientation.x = q.x();
    path.poses[1].pose.orientation.y = q.y();
    path.poses[1].pose.orientation.z = q.z();
    path.poses[1].pose.orientation.w = q.w();

    pub_path.publish(path);
}

void print_check(int _cnt, string msg) {
    if (_cnt%10 == 1) {
        std::cout << msg << std::endl;
    }
}