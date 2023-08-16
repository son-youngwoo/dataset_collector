#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <time.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
//경로주의
//#include <opencv/cv.h>
//#include <opencv/cv.hpp>
// #include <opencv/highgui.h>
// #include <opencv/highgui.hpp>

//
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <fstream>
#include <string>
#include <csv.hpp>

using namespace std;
using namespace csv;

std::string robot_namespace;


int id;
double world_x_init = 0;
double world_y_init = 0;
double world_z_init = 0;
double global_x_tar = 0;
double global_y_tar = 0;
int duration = 0;
double yaw_init = 0;
bool isSuccess = 0;
grid_map_msgs::GridMap elevation_map_realtime;
grid_map::GridMap elevation_map_global;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "dataset_checker");
    ros::NodeHandle nh;
        
    robot_namespace = argv[1]; // get robot_namespace from .launch 

    // ros::Rate loop_rate(10);

    try {
        CSVReader reader("path/to/your/file.csv");

        for (CSVRow& row : reader) {
            int value = stoi(row[1]);
            value += 10; // add 10 to the number

            row[1] = to_string(value); // update the value

            cout << row[0] << "," << row[1] << endl;
        }
    } catch (const CSVException& e) {
        cerr << "CSV parsing error: " << e.what() << endl;
        return 1;
    }

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}
