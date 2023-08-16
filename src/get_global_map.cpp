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

#include <dataset_collector/dataset.h>
#include <dataset_collector/dataset_rec_model.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

#include <fstream>
#include <string>
using namespace std;

grid_map::GridMap elevation_map_global;

std::string robot_namespace;
std::string bag_path;
std::string png_path;
// std::string _png_path;

void GetGlobalGrayImage(const grid_map_msgs::GridMap& msg, std::string _png_path)
{
  ros::NodeHandle nodeHandle_;
  std::string gridMapTopic_;
  std::string filePath_;

  srand((unsigned int)time(NULL));

  filePath_=_png_path;
  // filePath_="/home/son/Desktop/dataset/dataset3/map/terrain_3.png";

  grid_map::GridMap map;
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
  grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0, 0, image);
  bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
  std::cout << filePath_ << std::endl;
  sensor_msgs::Image ros_image;
  image.toImageMsg(ros_image);
}

void GetGlobalMap(std::string _bag_path) {
  // Open the bag file
  rosbag::Bag bag;
  bag.open(_bag_path, rosbag::bagmode::Read);
  // bag.open("/home/son/Desktop/dataset/dataset1/global_map_base.bag", rosbag::bagmode::Read);

  // Create a view for the desired topic
  rosbag::View view(bag, rosbag::TopicQuery("/elevation_mapping/elevation_map_raw"));

  // Iterate through the messages in the view
  for (const rosbag::MessageInstance& m : view)
  {
    // Extract the grid_map_msgs/GridMap message
    grid_map_msgs::GridMap::ConstPtr grid_map_msg = m.instantiate<grid_map_msgs::GridMap>();
    if (grid_map_msg != nullptr)
    {
      // Convert the message to a GridMap object
      grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, elevation_map_global);
    }
  }

  // Close the bag file
  bag.close();  
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "dataset_collector_rec_model");
  ros::NodeHandle nh;
  	
  robot_namespace = argv[1]; // get robot_namespace from .launch 

  nh.getParam("/get_global_map/bag_path", bag_path);
  nh.getParam("/get_global_map/png_path", png_path);

  GetGlobalMap(bag_path); // get grid_map::GridMap elevation_map_global

  // considering with difference between world frame base map and body frame base map
  // world base map data is positive
  // but robot body base map data is negative
  // robot's height is 0.62
  for (grid_map::GridMapIterator it(elevation_map_global); !it.isPastEnd(); ++it) {
    if (elevation_map_global.isValid(*it, "elevation_inpainted")) {
        elevation_map_global.at("elevation_inpainted", *it) -= 0.62;
    }
  }
  grid_map_msgs::GridMap elevation_map_global_msg; // grid_map_msgs
  grid_map::GridMapRosConverter::toMessage(elevation_map_global, elevation_map_global_msg);
  
  GetGlobalGrayImage(elevation_map_global_msg, png_path);

  return 0;
}
