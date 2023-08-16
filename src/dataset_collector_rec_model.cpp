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

grid_map_msgs::GridMap Elevation_Map_Copy;

#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

#include <fstream>
#include <string>
using namespace std;

int id;
double world_x_init = 0;
double world_y_init = 0;
double world_z_init = 0.6;
int left_yaw_max = 0;
int right_yaw_max = 0;

grid_map_msgs::GridMap elevation_map_realtime;

grid_map::GridMap elevation_map_global;

std::string robot_namespace;

grid_map::GridMap GetLocalMap(double _world_x_init, double _world_y_init, double _world_z_init);
void GetGlobalMap();

// void gridtoimage_pre(const grid_map_msgs::GridMap& msg, int num) //son for comparison with current method
// {
//   ros::NodeHandle nodeHandle_;
//   std::string gridMapTopic_;
//   std::string filePath_;

//   srand((unsigned int)time(NULL));

//   filePath_="/home/son/Desktop/dataset/dataset1/elevation_map_previous/num.png";
//   filePath_.replace(filePath_.find("num"), 3, std::to_string(num));
//   // nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/submap3"));

//   grid_map::GridMap map;
//   cv_bridge::CvImage image;
//   grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0, 0, image);
//   bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//   std::cout << filePath_ << std::endl;
//   sensor_msgs::Image ros_image;
//   image.toImageMsg(ros_image);
// }

// void gridtoimage_cur(const grid_map_msgs::GridMap& msg, int num) //son
// {
//   ros::NodeHandle nodeHandle_;
//   std::string gridMapTopic_;
//   std::string filePath_;

//   srand((unsigned int)time(NULL));

//   filePath_="/home/son/Desktop/dataset/dataset1/elevation_map_current/num.png";
//   filePath_.replace(filePath_.find("num"), 3, std::to_string(num));
//   // nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/submap3"));

//   grid_map::GridMap map;
//   cv_bridge::CvImage image;
//   grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0, 0, image);
//   bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//   std::cout << filePath_ << std::endl;
//   sensor_msgs::Image ros_image;
//   image.toImageMsg(ros_image);
// }
// void GetGlobalGrayImage(const grid_map_msgs::GridMap& msg) //son
// {
//   ros::NodeHandle nodeHandle_;
//   std::string gridMapTopic_;
//   std::string filePath_;

//   srand((unsigned int)time(NULL));

//   filePath_="/home/son/Desktop/dataset/dataset2/map/terrain_3.png";

//   grid_map::GridMap map;
//   cv_bridge::CvImage image;
//   grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0, 0, image);
//   bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//   std::cout << filePath_ << std::endl;
//   sensor_msgs::Image ros_image;
//   image.toImageMsg(ros_image);
// }

// void GetGrayImage_realtime(const grid_map_msgs::GridMap& msg, int num) //son for comparison with current method
// {
//   ros::NodeHandle nodeHandle_;
//   std::string gridMapTopic_;
//   std::string filePath_;

//   srand((unsigned int)time(NULL));

//   filePath_="/home/son/Desktop/dataset/dataset2/elevation_map_realtime/num.png";
//   filePath_.replace(filePath_.find("num"), 3, std::to_string(num));
//   // nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/submap3"));

//   grid_map::GridMap map;
//   cv_bridge::CvImage image;
//   grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0, 0, image);
//   bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//   std::cout << filePath_ << std::endl;
//   sensor_msgs::Image ros_image;
//   image.toImageMsg(ros_image);
// }

void GetGrayImage(const grid_map_msgs::GridMap& msg, int num) //son
{
  ros::NodeHandle nodeHandle_;
  std::string gridMapTopic_;
  std::string filePath_;

  srand((unsigned int)time(NULL));

  filePath_="/home/son/Desktop/dataset/dataset4/elevation_map_" + robot_namespace + "/num.png";
  filePath_.replace(filePath_.find("num"), 3, std::to_string(num));
  // nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/submap3"));

  grid_map::GridMap map;
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
  grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0, 0, image);
  bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
  std::cout << filePath_ << std::endl;
  sensor_msgs::Image ros_image;
  image.toImageMsg(ros_image);
}

void SaveDataset(int _id, double _world_x_init, double _world_y_init, double _left_yaw_max, double _right_yaw_max) {
    // create the output file stream

    ofstream file("/home/son/Desktop/dataset/dataset4/dataset_" + robot_namespace + ".csv", ios::app);

    // check if the file was opened successfully
    if (!file.is_open()) {
        cout << "Error opening file!" << endl;
    }

    // write the header row
    // file << "id,global_initial_x,global_initial_y,local_target_x,local_target_y,success_or_failure" << endl;

    // write some sample data
    file << _id << "," << _left_yaw_max << "," << _right_yaw_max << endl;

    // close the file
    file.close();

    cout << "Data saved to file!" << endl;
}

void msgCallbackDataset(const dataset_collector::dataset_rec_model& msg) {
    
    id = msg.id; // for dataset

    world_x_init = msg.world_x_init; // for GetLocalMap function
    world_y_init = msg.world_y_init; // for GetLocalMap function
    left_yaw_max = msg.left_yaw_max; // for dataset
    right_yaw_max = msg.right_yaw_max; // for dataset

    std::cout << "======================================" << std::endl;
    std::cout << "id: " << id << std::endl;
    std::cout << "world_x_init: " << world_x_init << std::endl;
    std::cout << "world_y_init: " << world_y_init << std::endl;
    std::cout << "left_yaw_max: " << left_yaw_max << std::endl;
    std::cout << "right_yaw_max: " << right_yaw_max << std::endl;

    grid_map::GridMap elevation_map_local;
    elevation_map_local = GetLocalMap(world_x_init, world_y_init, world_z_init); // grid_map
    grid_map_msgs::GridMap elevation_map_local_msg; // grid_map_msgs
    grid_map::GridMapRosConverter::toMessage(elevation_map_local, elevation_map_local_msg);

    GetGrayImage(elevation_map_local_msg, id);
    SaveDataset(id, world_x_init, world_y_init, left_yaw_max, right_yaw_max);

    // Use if you want compare local map from global map
    // with local map from realtime sensor data
    // grid_map::GridMap _elevation_map_realtime;
    // grid_map::GridMapRosConverter::fromMessage(elevation_map_realtime, _elevation_map_realtime, {"elevation"});

    // for (grid_map::GridMapIterator it(_elevation_map_realtime); !it.isPastEnd(); ++it) {
    // if (_elevation_map_realtime.isValid(*it, "elevation_inpainted")) {
    //     _elevation_map_realtime.at("elevation_inpainted", *it) -= world_z_init;
    //   }
    // }
    // grid_map_msgs::GridMap elevation_map_realtime_msg;
    // grid_map::GridMapRosConverter::toMessage(_elevation_map_realtime, elevation_map_realtime_msg);

    // GetGrayImage_realtime(elevation_map_realtime_msg, id);
}

void GetGlobalMap() {
  // Open the bag file
  rosbag::Bag bag;
  bag.open("/home/son/Desktop/dataset/dataset4/map/terrain_3.bag", rosbag::bagmode::Read);
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

grid_map::GridMap GetLocalMap(double _world_x_init, double _world_y_init, double _world_z_init) {

  // Define submap center and size
  grid_map::Position submap_center(_world_x_init, _world_y_init);  // center of submap in map coordinates
  grid_map::Length submap_size(1.4, 1.4);
  bool _isSuccess = true;
  
  // Extract submap
  grid_map::GridMap _elevation_map_local;
  _elevation_map_local = elevation_map_global.getSubmap(submap_center, submap_size, _isSuccess);

  for (grid_map::GridMapIterator it(_elevation_map_local); !it.isPastEnd(); ++it) {
    if (_elevation_map_local.isValid(*it, "elevation_inpainted")) {
        _elevation_map_local.at("elevation_inpainted", *it) -= _world_z_init;
    }
  }

  return _elevation_map_local;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dataset_collector_rec_model");
  ros::NodeHandle nh;
  // ros::Subscriber sub_dataset = nh.subscribe("/aidin81/dataset", 100, msgCallbackDataset);
  ros::Subscriber sub_dataset = nh.subscribe("dataset_rec_model", 100, msgCallbackDataset);
  	
  robot_namespace = argv[1]; // get robot_namespace from .launch 

  GetGlobalMap(); // get grid_map::GridMap elevation_map_global

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
