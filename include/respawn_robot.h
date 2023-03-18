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
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <dataset_collector/dataset.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <random>
#include <iostream>
#include <cmath>

#include <fstream>
#include <Eigen/Dense>



using namespace std;
using namespace Eigen;

int cnt = 0;
int cnt_resetsim = 0;
int cnt_resetworld = 0;
int cnt1 = 0;
double cnt_duration = 0;
int step = 2;
bool get_s_or_f = 0;
bool overturn = 0;
double x = 0;
double y = 0;
double z = 0;
double z_world = 0;
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

double world_x_tar = 0.0;
double world_y_tar = 0.0;
double trans_x_tar = 0.0;
double trans_y_tar = 0.0;
double global_x_tar = 0.0;
double global_y_tar = 0.0;

double rand_x_init = 0;
double rand_y_init = 0;



Matrix<double, 12, 1> JointAngle = MatrixXd::Zero(12, 1);


void Reset();


ros::Subscriber sub_bodypose;
ros::Subscriber sub_elevationmap;
ros::Subscriber sub_estimatedz;
ros::Subscriber sub_joint_angle;

ros::Publisher pub_path;
ros::Publisher pub_zerotorqueflag;
ros::Publisher pub_controlinput;
ros::Publisher pub_dataset;
ros::Publisher pub_xvel;
ros::Publisher pub_vel;
ros::Publisher pub_resetcontactflag;
ros::Publisher pub_randxy;
ros::Publisher pub_stateflag;

ros::ServiceClient client;
ros::ServiceClient resetworldClient; // robot model pose reset
ros::ServiceClient resetsimulationClient; // simulation reset

dataset_collector::dataset dataset;
grid_map_msgs::GridMap elevation_map_raw_copy;
grid_map_msgs::GridMap elevation_map_raw;
std_msgs::Int8 controlinput;

ofstream outfile("/home/son/Desktop/dataset/dataset1/terminal_output.txt");
ofstream outfile2("/home/son/joint_angle_correct.txt");

void msgCallbackEstimatedZ(const std_msgs::Float32::ConstPtr& msg) {
    z = msg->data;
    // ROS_ERROR("%f", z);
}

void msgCallbackBodyPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    x = msg->data[0];
    y = msg->data[1];
    z_world = msg->data[2];
    roll = msg->data[3];
    pitch = msg->data[4];
    yaw = msg->data[5];
}

void msgCallbackElevationMap(const grid_map_msgs::GridMap& msg)
{
    elevation_map_raw_copy = msg;
}


void msgCallbackJointPos_sim(const std_msgs::Float32MultiArray::ConstPtr& msgJoint)
{
	for (int i = 0; i < 12; i++){
		JointAngle[i] = msgJoint->data[i];
	}
}

void ROSInit(ros::NodeHandle& _nh)
{
    sub_bodypose = _nh.subscribe("/aidin81/BodyPose_sim", 10, msgCallbackBodyPose);
    sub_elevationmap = _nh.subscribe("/aidin81/elevation_mapping/elevation_map_raw", 10, msgCallbackElevationMap);
    sub_estimatedz = _nh.subscribe("/aidin81/estimatedz", 10, msgCallbackEstimatedZ);
    sub_joint_angle = _nh.subscribe("/aidin81/JointPos_sim", 1, msgCallbackJointPos_sim); //for gazebo


    pub_path = _nh.advertise<nav_msgs::Path>("/aidin81/Path", 100);
    pub_zerotorqueflag = _nh.advertise<std_msgs::Bool>("/aidin81/ZeroTorqueFlag", 100);
    pub_controlinput = _nh.advertise<std_msgs::Int8>("/aidin81/ControlInput", 100);
    pub_dataset = _nh.advertise<dataset_collector::dataset>("/aidin81/dataset", 100);
    pub_xvel = _nh.advertise<std_msgs::Float32>("/aidin81/xvel_target", 100);
    pub_vel = _nh.advertise<std_msgs::Float32MultiArray>("/aidin81/vel_target", 100);
    pub_randxy = _nh.advertise<std_msgs::Float32MultiArray>("/aidin81/randxy", 100);
    pub_resetcontactflag = _nh.advertise<std_msgs::Bool>("/aidin81/ResetContactFlag", 100);
    pub_stateflag = _nh.advertise<std_msgs::Bool>("/aidin81/StateFlag", 100);

    client = _nh.serviceClient<std_srvs::Empty>("/aidin81/elevation_mapping/clear_map");
    resetworldClient = _nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    resetsimulationClient = _nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    // ros::ServiceClient resetClient = nh.serviceClient<gazebo_msgs::ResetModel>("/gazebo/reset_model");

}


void PubResetContactFlag(bool _resetcontactflag){
    std_msgs::Bool resetcontactflag;
    resetcontactflag.data = _resetcontactflag;
    pub_resetcontactflag.publish(resetcontactflag);
}

void PubStateFlag(bool _stateflag){
    std_msgs::Bool stateflag;
    stateflag.data = _stateflag;
    pub_stateflag.publish(stateflag);
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

void PublishVelocity(double _xvel) {
    std_msgs::Float32 xvel_target;
    xvel_target.data = _xvel;
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

    // // create a random number generator engine
    // std::random_device rd_init;
    // std::mt19937 gen_init(rd_init());

    // // create a distribution that generates random double numbers in [0.0, 1.0)
    // std::uniform_real_distribution<double> dis_init(-17.0, 17.0);

    // rand_x_init = -8;//dis_init(gen_init);
    // rand_y_init = 12;//dis_init(gen_init);
    
    // ROS_ERROR("rand_x_init: %f", rand_x_init);

    // if(((-9.0 < rand_x_init) && (rand_x_init <= 0.0)) && ((8.0 < rand_y_init) && (rand_y_init < 17.0))) {  
    //     ROS_ERROR("hererher");
    //     std::random_device rd_init2;
    //     std::mt19937 gen_init2(rd_init2());
        
    //     std::uniform_real_distribution<double> dis_init2(-17.0, 17.0);

    //     rand_x_init = dis_init2(gen_init2);
    //     rand_y_init = dis_init2(gen_init2);
    // }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_init(-16.5, 16.5);


    do {
        rand_x_init = dis_init(gen);
        // ROS_ERROR("rand_x_init: %f", rand_x_init);
    } while (rand_x_init >= -9.0 && rand_x_init <= 0.0);

    do {
        rand_y_init = dis_init(gen);
        // ROS_ERROR("rand_y_init: %f", rand_y_init);
    } while (rand_y_init >= 8.0 && rand_y_init <= 16.5);

    tf2::Quaternion q1;
    q1.setRPY(0, 0, 1.57);

    gazebo_msgs::ModelState modelState;
    modelState.model_name = "aidin81";  // Replace with your robot's name
    modelState.pose.position.x = rand_x_init;   // Replace with your robot's starting position
    modelState.pose.position.y = rand_y_init;
    modelState.pose.position.z = 1.5;
    modelState.pose.orientation.x = 0;  // Replace with your robot's starting orientation
    modelState.pose.orientation.y = 0;
    modelState.pose.orientation.z = 0;
    modelState.pose.orientation.w = 1;
    // modelState.pose.orientation.x = q1.x();  // Replace with your robot's starting orientation
    // modelState.pose.orientation.y = q1.y();
    // modelState.pose.orientation.z = q1.z();
    // modelState.pose.orientation.w = q1.w();
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

    std_msgs::Float32MultiArray randxy;
    randxy.data.push_back(rand_x_init);
    randxy.data.push_back(rand_y_init);
    pub_randxy.publish(randxy);
}

void PublishPath(int _cnt_path){
    if (_cnt_path == 1) {
        //std::cout << "publish target position" << std::endl;        

        x_init = x;
        y_init = y;
        
        // std::cout << "global_initial_x: " << x_init << std::endl;        // test
        // std::cout << "global_initial_y: " << y_init << std::endl;        // test

        // Define the random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_angle(-M_PI, M_PI);
        std::uniform_real_distribution<> dis_radius(min_radius, max_radius);
        
        // Generate a random angle and distance
        double angle = dis_angle(gen);
        double radius = dis_radius(gen);
        
        rand_x_tar = x_init + radius * std::cos(angle); // world base
        rand_y_tar = y_init + radius * std::sin(angle); // world base

        pos_x = cos(-yaw_init)*(rand_x_tar - x)-sin(-yaw_init)*(rand_y_tar - y);
        pos_y = sin(-yaw_init)*(rand_x_tar - x)+cos(-yaw_init)*(rand_y_tar - y);

        // yaw_target = atan2(rand_y_tar - y, rand_x_tar - x); // theta based world frame = angle 
        yaw_target = atan2(pos_y, pos_x); // theta based world frame = angle + yaw_init

        // world_x_tar = radius * std::cos(angle);
        // world_y_tar = radius * std::sin(angle);

        // trans_x_tar = world_x_tar + x_init;
        // trans_y_tar = world_y_tar + y_init;

        // global_x_tar = cos(-yaw_init)*trans_x_tar - sin(-yaw_init)*trans_y_tar;
        // global_y_tar = sin(-yaw_init)*trans_x_tar + cos(-yaw_init)*trans_y_tar;

        // yaw_target = atan2(global_y_tar, global_x_tar);

        yaw_target_deg = abs(yaw_target/M_PI*180);

        // num_div = yaw_target_deg / 90; // 이렇게 주니까 로봇이 이상하네..?
        num_div = yaw_target_deg / 4;
        yaw_target_dis = yaw_target / num_div;

        if(std::isnan(yaw_target_dis) != 0) {
            yaw_target_dis = 0;
            ROS_ERROR("yaw_target_dis is Nan!!!!");
        }

        std::cout << "  > world_x_tar: " << world_x_tar << " / world_y_tar: " << world_y_tar << std::endl;
        outfile << "  > world_x_tar: " << world_x_tar << " / world_y_tar: " << world_y_tar << "\n";

        std::cout << "  > trans_x_tar: " << trans_x_tar << " / trans_y_tar: " << trans_y_tar << std::endl;
        outfile << "  > trans_x_tar: " << trans_x_tar << " / trans_y_tar: " << trans_y_tar << "\n";

        std::cout << "  > global_x_tar: " << global_x_tar << " / global_y_tar: " << global_y_tar << std::endl;
        outfile << "  > global_x_tar: " << global_x_tar << " / global_y_tar: " << global_y_tar << "\n";

        std::cout << "  > yaw_target: " << yaw_target << std::endl;
        outfile << "  > yaw_target: " << yaw_target << "\n";
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
        outfile << msg << "\n";
    }
}

