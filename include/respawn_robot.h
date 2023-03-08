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


int respawn_flag = 0;
bool get_s_or_f = 0;
bool overturn = 0;
bool arrive = 0;
double x = 0;
double y = 0;
double z = 0;
double x_thres = 0.05;
double y_thres = 0.05;
double roll_limit = 0.7;
double pitch_limit = 0.7;
double roll = 0;
double pitch = 0;
double yaw = 0;
int data_id = 0;
bool s_or_f;
int cnt = 0;
double timer0 = 0;
double timer1 = 0;
double timer2 = 0;
double _timer2 = 0;
double timer3 = 0;
double timer_reset = 0;
double yaw_target = 0;
double _yaw_target = 0;
double __yaw_target = 0;
double yaw_target_dis = 0;
int k = -1;
double rand_x_tar = 0;
double rand_y_tar = 0;
double x_init = 0;
double y_init = 0;
int num_div = 0;
double yaw_target_deg = 0;
double add_x = 0;
double add_y = 0;
double d = 0;
double R_success = 0;
double xvel_target = 0;
double yvel_target = 0;
bool reset = 0;
bool ov = 0;

void Reset();

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
std_msgs::Int8 controlinput;

void msgCallbackBodyPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    x = msg->data[0];
    y = msg->data[1];
    z = msg->data[2];
    roll = msg->data[3];
    pitch = msg->data[4];
    yaw = msg->data[5];

    // if (ov == 0) {
    //     if(roll > roll_limit || pitch > pitch_limit) {
    //         overturn = 1;
    //     }
    //     else {
    //         overturn = 0;
    //     }
    // }
    // d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
    // R_success = 0.15;
    //  if ( d < R_success) // arrive or not?
    // {
    //     arrive = 1;
    // }
    // else {
    //     arrive = 0;
    // }

    // if(overturn  == 1 && arrive == 1) { // failure
    //     get_s_or_f = 1;
    //     s_or_f = 0;
    // }
    // else if(overturn == 1 && arrive == 0) { // reset      
    //     // respawn_flag = 5;
    //     // reset = 1;
    //     ov = 1;
    //     respawn_flag = 5;
    // }
    // else if(overturn == 0 && arrive == 1) { // success
    //     get_s_or_f = 1;
    //     s_or_f = 1;
    // }
    // else if(overturn == 0 && arrive == 0) { // wait result
    //     get_s_or_f = 0;
    // }
}

// void subscriberCallback1 (const  grid_map_msgs::GridMap& msg1)
// {   Elevation_Map_Copy = msg1;
//     mappingstart = 1;}

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

void CrawlMode() {
    controlinput.data = 3;
    pub_controlinput.publish(controlinput);
    std::cout << "command crawl mode"  << std::endl;  
}

void InitializeSensor() {
    std::cout << "initialize sensor" << std::endl;
    controlinput.data = 1;
    pub_controlinput.publish(controlinput);
}

void InitializeIMU() {
    std::cout << "initialize imu" << std::endl;
    controlinput.data = 2;
    pub_controlinput.publish(controlinput);
}

void StandMode() {
    std::cout << "command stand mode"  << std::endl;
    controlinput.data = 4;
    pub_controlinput.publish(controlinput);
}

void MPCMode() {
    std::cout << "cammand mpc mode" << std::endl;
    controlinput.data = 5;
    pub_controlinput.publish(controlinput); 
}

void ResetElevationMap() {
    std::cout << "elevation map reset" << std::endl;
    std_srvs::Empty srv;
    if (client.call(srv)) {
        ROS_INFO("Elevation map cleared.");
    } else {
        ROS_ERROR("Failed to call service /aidin81/elevation_mapping/clear_map");
    }
}

void GetElevationMap() {
    std::cout << "get elevation map" << std::endl;        
    dataset.elevation_map_raw = elevation_map_raw_copy;
}

void PublishZeroVelocity() {
    std_msgs::Float32 xvel_target;
    xvel_target.data = 0;
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

void Stop() {         
    CrawlMode();
    PublishZeroVelocity();
    PublishZeroPath();
}

void Respawn() {
    std::cout << "respawn ..." << std::endl;        

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
        std::cout << "respawn x: " << rand_x_init << " y: " << rand_y_init << std::endl;        
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
    }
}

void SetYawTarget() {
    std::cout << "publish target position" << std::endl;        

    // Define the minimum and maximum radii
    double min_radius = 0.8;
    double max_radius = 1.0;
    double centerX = x;
    double centerY = y;

    x_init = x;
    y_init = y;

    dataset.global_initial_x = x_init;
    dataset.global_initial_y = y_init;
    
    // Define the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_angle(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> dis_radius(min_radius, max_radius);
    
    // Generate a random angle and distance
    double angle = dis_angle(gen);
    double radius = dis_radius(gen);
    
    // Calculate the x and y coordinates
    rand_x_tar = centerX + radius * std::cos(angle); // world base
    rand_y_tar = centerY + radius * std::sin(angle); // world base

    // target x,y dataset 저장.
    dataset.local_target_x = rand_x_tar - centerX; // robot base
    dataset.local_target_y = rand_y_tar - centerY; // robot base

    yaw_target = atan2(rand_y_tar - y, rand_x_tar - x); // theta based world frame

    yaw_target_deg = abs(yaw_target/M_PI*180);

    num_div = yaw_target_deg / 6;
    
    yaw_target_dis = yaw_target / num_div;
}

void PublishPath(double _x_init, double _y_init, double _rand_x_tar, double _rand_y_tar){

}

double GetYawTarget(int _num_div, double _yaw_target_dis) {
    k++;

    if(k < num_div + 1) {
        __yaw_target = _yaw_target_dis*k;
    }
    else {
        __yaw_target = _yaw_target_dis*_num_div;
    }

    return __yaw_target;
}