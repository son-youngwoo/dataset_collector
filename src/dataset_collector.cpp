// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <boost/thread.hpp>
// #include <string>
// #include <iostream>
// #include <stdio.h>
// #include <math.h>
// #include <cstdlib>
// #include <ctime>
// #include <time.h>

// // Eigen
// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

// //
// #include <sensor_msgs/image_encodings.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/imgcodecs.hpp>
// //경로주의
// //#include <opencv/cv.h>
// //#include <opencv/cv.hpp>
// // #include <opencv/highgui.h>
// // #include <opencv/highgui.hpp>

// //
// #include <nav_msgs/Odometry.h>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_msgs/GridMap.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// // tf2
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Transform.h>

// // aidin_msgs
// #include <aidin_msgs/one.h>
// #include <aidin_msgs/two.h>
// #include <aidin_msgs/three.h>
// #include <aidin_msgs/four.h>
// #include <aidin_msgs/twel.h>
// #include <aidin_msgs/pt_array.h>
// #include <aidin_msgs/thir.h>

// #include <dataset_collector/dataset.h>

// #define estimationRadius_normal 0.10//0.08
// #define estimationRadius_roughness  0.10//0.06//0.08//0.08//0.06
// #define criticalValue_roughness  0.05//0.05//0.01//0.007//0.02//0.02
// #define roughness_threshold 0.2

// #define ROS_RED_STREAM(SSS) ROS_INFO_STREAM("\033[31;1m" << SSS << "\033[0m")

// // ros::Publisher submap_pub_;
// // ros::Publisher pub_roughness;
// // ros::Publisher pub_height;
// // ros::Publisher normal;
// // ros::Publisher red;
// // ros::Publisher pub_foothold;
// // ros::Publisher pub_img;

// aidin_msgs::two GP;
// // const int numrow = 100;
// // const int numcol = 2;
// aidin_msgs::pt_array roughness_;
// aidin_msgs::pt_array height_;
// aidin_msgs::four footholdpredict;
// aidin_msgs::four target_pos, normal_pos;
// aidin_msgs::four temp,temp_;
// aidin_msgs::three spot;
// visualization_msgs::MarkerArray marker;
// grid_map_msgs::GridMap Elevation_Map_Copy;

// int idx_max;
// int sunseo = 0;
// int mappingstart;
// double cnt = 0;

// #include <std_msgs/Float32.h>
// #include <std_msgs/Int64.h>

// #include <fstream>
// #include <string>
// using namespace std;

// int id;
// float global_initial_x;
// float global_initial_y;
// float local_target_x;
// float local_target_y;
// float yaw_init;
// float yaw_target;
// float duration;
// bool s_or_f;

// void TargetFoot(const aidin_msgs::four& msg)
// {
// 		target_pos.a = msg.a;
// 		target_pos.b = msg.b;
// 		target_pos.c = msg.c;

// 		//printf("posy,posz : (%.3f, %.3f)\n",target_pos.b,target_pos.c );

// }

// void Footprint(visualization_msgs::MarkerArray *marker, aidin_msgs::four msg)
//     {
//         visualization_msgs::Marker Foot;
// 		float x, y, z;
// 		std::string str;
//         int i = 0;

// 		Foot.header.frame_id = "odom";
// 		Foot.header.stamp = ros::Time::now();


// 		Foot.type = 2;


// 		Foot.scale.x = 0.05;
// 		Foot.scale.y = 0.05;
// 		Foot.scale.z = 0.05;
		
// 		Foot.lifetime = ros::Duration();

//         Foot.ns = str;
// 		//Foot.id = i;
// 		Foot.id = 55;

// 		Foot.pose.position.x = msg.a;
// 		Foot.pose.position.y = msg.b;
// 		Foot.pose.position.z = msg.c;
// 		Foot.color.r = 1;
//         Foot.color.g = 0;//1;
//         Foot.color.b = 0;//0.5;
//         Foot.color.a = 1;

// 		Foot.id = 1;

// 		marker->markers.push_back(Foot);
 
//         Foot.lifetime = ros::Duration();

//     }

// void Footprint2(visualization_msgs::MarkerArray *marker, aidin_msgs::four msg)
//     {
//         visualization_msgs::Marker Foot;
// 		float x, y, z;
// 		std::string str;
//         int i = 0;

// 		Foot.header.frame_id = "odom";
// 		Foot.header.stamp = ros::Time::now();

// 		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// 		Foot.type = 2;

// 		// Set the scale of the marker -- 1x1x1 here means 1m on a side
// 		Foot.scale.x = 0.05;
// 		Foot.scale.y = 0.05;
// 		Foot.scale.z = 0.05;
		
// 		Foot.lifetime = ros::Duration();

//         Foot.ns = str;
// 		//Foot.id = i;
// 		Foot.id = 55;

// 		Foot.pose.position.x = msg.a;
// 		Foot.pose.position.y = msg.b;
// 		Foot.pose.position.z = msg.c;
// 		Foot.color.r = 0;
//         Foot.color.g = 1;//1;
//         Foot.color.b = 0;//0.5;
//         Foot.color.a = 1;

// 		Foot.id = 1;

// 		marker->markers.push_back(Foot);
 
//         Foot.lifetime = ros::Duration();

//     }

// void computeWithArea(grid_map::GridMap& map, 
//                                     const std::string& inputLayer, 
//                                     const std::string& outputLayersPrefix)//, Eigen::Vector2d center_) //Vector2d Position 筌�슪�숋옙�뉗퐷占쎄랜肉덌옙�숈삕占썩뫅��. 
// {
//   // ! Radius of submap for normal vector estimation.
//   double estimationRadius_ = estimationRadius_normal;
//   Eigen::Vector3d normalVectorPositiveAxis_ = grid_map::Vector3::UnitZ();
//   std::string inputLayer_ = "elevation";
//   std::string outputLayersPrefix_ = "surface_normal_";

//   for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

//     // Requested position (center) of circle in map.
//     grid_map::Position center;
//     map.getPosition(*iterator, center);
//     // Prepare data computation.
//     const int maxNumberOfCells = pow(ceil(2 * estimationRadius_ / map.getResolution()), 2);//+1筌�옙李볟㎤�우퐷占쎈챸�륅㎖占쎌컝占쎈�愿쒎㎤遺우컞筌�꼹��뜝�꾩퐷占쎄램�득쾮�ш킐冶⑤뗄李됵㎖占쎌컭冶⑨옙
//     Eigen::MatrixXd points(3, maxNumberOfCells);
//     size_t nPoint = 0;
   
//     // Gather surrounding data.
//     size_t nPoints = 0;

//     for (grid_map::CircleIterator iterator(map, center, estimationRadius_); !iterator.isPastEnd(); ++iterator) {
// //    if (!map.isValid(*iterator, inputLayer_)) continue;
//       grid_map:: Position3 point;
//       map.getPosition3(inputLayer_, *iterator, point);
//       points.col(nPoints) = point;

//       nPoints++;
//     }
//     //printf("nPoints: %.3f       ",nPoints);

//     // nPoints = nPoints + 1 ;//temp
//     if(! (nPoints <= 0)){
//       points.conservativeResize(3, nPoints); // TODO Eigen version? //筌�옙李볟㎤�곸뒇�좎럩肄좑쭪�밤럮
    
//       // Compute Eigenvectors.
//       const grid_map::Position3 mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);///////
//       const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;
    
//       const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
//       grid_map::Vector3 eigenvalues = grid_map::Vector3::Ones();
//       Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
//       // Ensure that the matrix is suited for eigenvalues calculation.
//       if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
//         const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
//         eigenvalues = solver.eigenvalues().real();
//         eigenvectors = solver.eigenvectors().real();
//       } 
//       else {
//         ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
//         // Use z-axis as default surface normal. // TODO Make dependend on surfaceNormalPositiveAxis_;
//         eigenvalues.z() = 0.0;
//       }
//       // Keep the smallest Eigenvector as normal vector.
//       int smallestId(0);
//       double smallestValue(std::numeric_limits<double>::max());
//       for (int j = 0; j < eigenvectors.cols(); j++) {
//         if (eigenvalues(j) < smallestValue) {
//           smallestId = j;
//           smallestValue = eigenvalues(j);
//         }
//       }
  
//       grid_map::Vector3 eigenvector = eigenvectors.col(smallestId);
//       if (eigenvector.dot(normalVectorPositiveAxis_) < 0.0) eigenvector = -eigenvector;
//   //    map.getIndex(center,*index);
      
//       //float normalX, normalY, normalZ;

//       //normalX = eigenvector.x();
//       //normalY = eigenvector.y();
//       //normalZ = eigenvector.z();
//       map.at(outputLayersPrefix_ + "x", *iterator) = eigenvector.x();
//       map.at(outputLayersPrefix_ + "y", *iterator) = eigenvector.y();
//       map.at(outputLayersPrefix_ + "z", *iterator) = eigenvector.z();
//       }
//       else ;
//   }
// }

// bool update_total(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut,aidin_msgs::pt_array& msg)
// {
//   unsigned char start_flag;
//   double update_time_start = ros::Time::now().toSec();
//  //ROS_INFO("total start!");
//   // Add new layers to the elevation map.
//   std::string type_ = "traversability_total";
//   mapOut = mapIn;
//   mapOut.add(type_);

//   double total,totalMax,slope,roughness,step_height;
//   double criticalValue_=0.5;//0.6

//     static uint32_t nPoint = 0 ;
//     static uint32_t nPoint_collision = 0;
//     aidin_msgs::pt_roughness msg_;



//   // First iteration through the elevation map.
//   for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
//     if (!mapOut.isValid(*iterator, "slope"))
//       continue;
//     if (!mapOut.isValid(*iterator, "traversability_roughness"))
//       continue;
// //    if (!mapOut.isValid(*iterator, "step_height"))
// //      continue;
//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
    
//     double x,y,z;
//     x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
//     msg_.x = x; msg_.y = y; msg_.z = z;

//     slope = mapOut.at("slope", *iterator);
//     roughness = mapOut.at("traversability_roughness", *iterator);
// //    step_height = mapOut.at("step_height", *iterator);

//     total = (0.5 * slope + 0.5 * roughness + 0.0 * step_height);

//     msg_.r_roughness = total;
//     //total = (0.2 * slope + 0.3*roughness + 0.0*step_height);
//     if(total < criticalValue_){mapOut.at(type_,*iterator) =  0.2+0.8*total/criticalValue_; msg_.roughness = 0.2;}//total
//     else {mapOut.at(type_,*iterator) = 1;      msg_.roughness = 1.0;}	
//     msg.array.push_back(msg_);
   

//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else; 
//     nPoint = nPoint +1;
//     if (total > totalMax) totalMax = total;
//     else ;
//   }
//     msg.nPoint = nPoint;
//     // Collision_.nPoint = nPoint_collision;

//     nPoint = 0; nPoint_collision = 0;
//   ROS_DEBUG("slope total = %f", totalMax);
//   return true;
// }

// bool update_roughness1(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)//,Eigen::Vector2d center_)
// {
//     std::string type_ = "traversability_roughness";
//     double estimationRadius_ = estimationRadius_roughness;
//     double criticalValue_ = criticalValue_roughness;

//     // Add new layer to the elevation map.
//     mapOut = mapIn;
//     mapOut.add(type_);
//     double roughnessMax = 0.0;
//     unsigned char start_flag = 0;
//     static uint32_t nPoint = 0 ;

//     for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {
    
//     // Check if this is an empty cell (hole in the map).
//     //if (!mapOut.isValid(*iterator, "surface_normal_x")) continue;
  
//     // Prepare data computation.
//     const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));//+1筌�옙李볟㎤�우퐷占쎈챸�륅㎖占쎌컝占쎈�愿쒎㎤遺우컞筌�꼹��뜝�꾩퐷占쎄램�득쾮�ш킐冶⑤뗄李됵㎖占쎌컭冶⑨옙
//     Eigen::MatrixXd points(3, maxNumberOfCells);
//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
    
//     double x,y,z;
//     x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
//     //if(!(-0.01<x)&&(x<0.01))
//     {
    
//     // Requested position (center) of circle in map.
//     grid_map::Position center;
//     center(0,0) = xyz(0,0);
//     center(1,0) = xyz(1,0);
//     //mapOut.getPosition(*iterator, center);

//     // Gather surrounding data.
//     size_t nPoints = 0;
//     double h_max = -1, h_min = 100;
//     for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
//         !submapIterator.isPastEnd(); ++submapIterator) 
//     {
//       if (!mapOut.isValid(*submapIterator, "elevation")) continue;
        
//       Eigen::Vector3d point;
//       mapOut.getPosition3("elevation", *submapIterator, point);
//       points.col(nPoints) = point;

//       h_max = std::max(h_max, point(2,0));
//       h_min = std::min(h_min, point(2,0));
//       nPoints++; 
//     }

//     if(! (nPoints <= 0)){
//     const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);
    
//     double normalX = mapOut.at("surface_normal_x", *iterator);
//     double normalY = mapOut.at("surface_normal_y", *iterator);
//     double normalZ = mapOut.at("surface_normal_z", *iterator);
//     double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
//     double sum = 0.0;
    
//     for (int i = 0; i < nPoints; i++) 
//     {
//       double dist = 1*normalX*points(0,i) + 1*normalY*points(1,i) + 1*normalZ*points(2,i) - planeParameter + 0.7*sqrt(1-normalZ*normalZ)*abs(points(2,i)-mean.z());//modifyed  0.7
//       sum += pow(dist,2);
//     }
//     double roughness = sqrt(sum / (nPoints+1 ));//-1

    
//     if (roughness < criticalValue_) 
//     {
//       mapOut.at(type_, *iterator) = 0.2+ 0.8*(roughness / criticalValue_);//1.0 - roughness / criticalValue_;

//     }
//     else 
//     {
//       mapOut.at(type_, *iterator) = 1.0;//0.0; //modify


//     }

//     //msg_.header.stamp = ros::Time::now();
//     //mapOut.at(type_, *iterator) = roughness;

//     if (roughness > roughnessMax) roughnessMax = roughness;
//     else ;
    
//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else; 

//     }
//       else ;
//     }
      
//   }

//     ROS_DEBUG("roughness max = %f", roughnessMax);
//     return true;
// }

// bool update_edge(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut,aidin_msgs::pt_array& msg)//,Eigen::Vector2d center_)
// {
//     std::string type_ = "traversability_roughness";
//     double estimationRadius_ = estimationRadius_roughness;
//     double criticalValue_ = criticalValue_roughness;

//     mapOut = mapIn;
//     mapOut.add(type_);
//     double roughnessMax = 0.0;
//     unsigned char start_flag = 0;

//     static uint32_t nPoint = 0 ;
//     aidin_msgs::pt_roughness msg_;
//     for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {
//     const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));//+1해주었었음
//     Eigen::MatrixXd points(3, maxNumberOfCells);
//     //3 행 maxNumberOfCells 렬의 행렬. 
//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
//       double x,y,z;
//       x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
// //      printf("z:%f\n",z);
//       //if(!(-0.01<x)&&(x<0.01))
//       {msg_.x = x; msg_.y = y; msg_.z = z;

//     grid_map::Position center;
//     center(0,0) = xyz(0,0);
//     center(1,0) = xyz(1,0);
//     //mapOut.getPosition(*iterator, center);

//     // Gather surrounding data.
//     size_t nPoints = 0;
//     double h_max = -1, h_min = 100;
//     for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
//         !submapIterator.isPastEnd(); ++submapIterator) 
//     {
//       if (!mapOut.isValid(*submapIterator, "elevation")) continue;
        
//       Eigen::Vector3d point;
//       mapOut.getPosition3("elevation", *submapIterator, point);
//       points.col(nPoints) = point;

//       h_max = std::max(h_max, point(2,0));
//       h_min = std::min(h_min, point(2,0));


//       nPoints++; 


//     }

//     if(! (nPoints <= 0)){
//     const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);
    
//     double sum = 0.0;
//     for (int i = 0; i < nPoints; i++) 
//     {
//       double dist = fabs(mean.z()-points(2,nPoints));;
//       sum += dist;
//     }
//     double roughness = h_max - h_min;    
// 	//ROS_INFO("roughness: %5f\n",roughness);     
//     msg_.r_roughness = roughness;
//     if (roughness < criticalValue_) //modify
//     {
//       mapOut.at(type_, *iterator) = 0.2;//1.0 - roughness / criticalValue_;
//       msg_.roughness = 0.2; 
// //      printf("ture\n");
//     }
//     else
//     {
//       mapOut.at(type_, *iterator) = 1.0;//0.0; //modify
//       msg_.roughness = 1.0;
//       printf("false\n");
//     }
//     //msg_.header.stamp = ros::Time::now();
//     //mapOut.at(type_, *iterator) = roughness;
//     msg.array.push_back(msg_);
//     if (roughness > roughnessMax) roughnessMax = roughness;
//     else ;

//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else; 
//     nPoint = nPoint +1;
//       }
//       else ;
//       }
      
//   }

//     msg.nPoint = nPoint;
//     nPoint = 0;
//     ROS_DEBUG("roughness max = %f", roughnessMax);
//     return true;
// }

// // void update_heightedge(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
// // {       
// //     ROS_INFO("normal Vector Start!");
// //         if(update_edge(submap,submap,msg)) ROS_RED_STREAM("Complete update_heightedge in Enable_Traverse");
// //         else ROS_WARN_STREAM("Not Complete update_traverse in Enable_Traverse");
// // }

// bool update_slope(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)
// {
//   double update_time_start = ros::Time::now().toSec();

//   std::string type_ = "slope";
//   unsigned char start_flag=0;
//   // Add new layer to the elevation map.
//   mapOut = mapIn;
//   mapOut.add(type_);

//   double slope, slopeMax = 0.0,slopeMin = 0.5;
//   double criticalValue_ = 0.78;
  
//   for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {

//     // Check if there is a surface normal (empty cell).
//     if (!mapOut.isValid(*iterator, "surface_normal_z")) continue;

//     // Compute slope from surface normal z
//     slope = acos(mapOut.at("surface_normal_z", *iterator));

//     if (slope < criticalValue_) {
//       mapOut.at(type_, *iterator) = 0.2 + 0.8*(slope/criticalValue_);//1.0 - slope / criticalValue_;
//     }
//     else {
//       mapOut.at(type_, *iterator) = 1.0;//0.0;
//     }

//     if (slope > slopeMax) slopeMax = slope;
//     if (slope < slopeMin) slopeMin = slope;
    
//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else;


//   }
//   //ROS_INFO("Slope_MIN/MAX :%.3f, %.3f",slopeMax,slopeMin);
//   ROS_DEBUG("slope max = %f", slopeMax);

//   return true;
// }

// bool update_normalvector(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)//, Eigen::Vector2d center_) 
// {
//   //! Normal vector positive axis.
//   Eigen::Vector3d normalVectorPositiveAxis_ = grid_map::Vector3::UnitZ();

//   //! Input layer name.
//   std::string inputLayer_ = "elevation";

//   //! Output layer name.
//   std::string outputLayersPrefix_ = "surface_normal_";
 
//   //ROS_INFO_THROTTLE(0.3, "update Normal_vector!");
//   std::vector<std::string> normalVectorsLayers;
//   normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
//   normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
//   normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

//   mapOut = mapIn;
//   for (const auto& layer : normalVectorsLayers) mapOut.add(layer);
 
//     computeWithArea(mapOut, inputLayer_, outputLayersPrefix_);

//   return true;
// }

// void update_traverse(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
// {       
//     if(update_normalvector(submap,submap))
//     {
// 	if(update_slope(submap,submap)){if(update_roughness1(submap,submap)) {if(update_total(submap,submap,msg))ROS_RED_STREAM("Complete update_traverse in Enable_Traverse");}}
//         else ;
//     }
//     else ; 
// }

// grid_map::GridMap getSubmapMessage(grid_map::GridMap& original_gridmap, //const grid_map::GridMap& original_gridmap, 
//                                     const grid_map::Position& position,
//                                     const grid_map::Length& length,
//                                     const std::vector<std::string>& layers,
//                                     const bool useRaw, bool& isSuccess)
// {
//     grid_map::Index index;
  
//     grid_map::GridMap subMap;
//     if (useRaw) 
//     {   
//         subMap = original_gridmap.getSubmap(position, length, index, isSuccess);
//     } else ROS_INFO("useRaw is false.");
//     grid_map_msgs::GridMap message;
//     grid_map::GridMapRosConverter::toMessage(subMap, layers, message);

//     return subMap; 
// }

// void foothold_detect(const aidin_msgs::pt_array& msg,
//                                     const aidin_msgs::three& G_P_F, 
//                                     aidin_msgs::four& msg_)
// {
//     //ROS_INFO("come in foothold");
//     int nPoint_ = (msg.nPoint); // array 갯수
//     unsigned char continue_check = 0,num = 0;
//     float x = G_P_F.a;
//     float y = G_P_F.b;
//     float z = G_P_F.c;
//     //msg_.d = G_P_F.d;
//     //float dist ;
//     std::vector<double> v_length_dist, v_origin_dist, v_foot_dist; 
//     std::vector<double> v_length_score, v_origin_score, v_foot_score, v_total_score; 
//     Eigen::Vector3d G_P_ss_;
//     G_P_ss_(0) = G_P_F.a;
//     G_P_ss_(1) = G_P_F.b;
//     G_P_ss_(2) = G_P_F.c;
//     // if(nPoint_ < 5) 
//     // {
//     //     printf("- Submap Not Complete :  nPoint_ : %d !!!!!!\n", nPoint_);
//     //     msg_.a = x; msg_.b = y; msg_.c = 0; msg_.d = FailMapData;

//     // }
//     // else
//     // {
//         for(int i = 0; i < nPoint_; i++)
//         {
//             float dist1, dist2, dist3;
//             //if((-0.05<msg.array[i].z)&&(msg.array[i].z<0.05))
//             {
//             dist1 = sqrt(pow(msg.array[i].x - G_P_ss_(0), 2) + pow(msg.array[i].y - G_P_ss_(1), 2) + pow(msg.array[i].z - G_P_ss_(2), 2));;
//             v_origin_dist.push_back(dist1);
//             dist2 = sqrt(pow(msg.array[i].x - G_P_ss_(0), 2) + pow(msg.array[i].y - G_P_ss_(1), 2));
//             v_length_dist.push_back(dist2);
//             dist3 = sqrt((msg.array[i].x-x)*(msg.array[i].x-x)
//                     +(msg.array[i].y-y)*(msg.array[i].y-y));
//             v_foot_dist.push_back(dist3);
//             }
//         }
//         double origin_max = *max_element(v_origin_dist.begin(),v_origin_dist.end());
//         double origin_min = *min_element(v_origin_dist.begin(),v_origin_dist.end());
//         printf("-origin_max     : %.3f           min   : %.3f       \n",origin_max , origin_min);
//         double length_max = *max_element(v_length_dist.begin(),v_length_dist.end());
//         double length_min = *min_element(v_length_dist.begin(),v_length_dist.end());

//         double foot_max = *max_element(v_foot_dist.begin(),v_foot_dist.end());
//         double foot_min = *min_element(v_foot_dist.begin(),v_foot_dist.end());
        
//         for(int i = 0; i < nPoint_; i++)
//         {  
//           //v_origin_score.push_back( ( origin_max ) - ( v_origin_dist[i] ) );
//           //v_length_score.push_back( ( v_length_dist[i] ) - (length_max ) ); //max length
//           //v_foot_score.push_back( (foot_max ) - ( v_foot_dist[i] / (foot_max-foot_min)) );
//           //v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + v_foot_score[i] );
        
//           // 나누기는 여기밖에......
//           v_origin_score.push_back( ( origin_max / (origin_max-origin_min)) - ( v_origin_dist[i] / (origin_max-origin_min)) );
//           v_length_score.push_back( ( v_length_dist[i] / (length_max-length_min)) - (length_max / (length_max-length_min)) ); //max length
//           v_foot_score.push_back( (foot_max / (foot_max-foot_min)) - ( v_foot_dist[i] / (foot_max-foot_min)) );
//           v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + 2*v_foot_score[i] );
// //          std::cout <<"origin"<<v_origin_score<<"length"<<v_length_score;
// //        printf("-origin : %.3f,%.3f,%.3f,%.3f    length : %.3f,%.3f,%.3f,%.3f   foot : %.3f   total : %.3f \n",v_origin_score, v_length_score, v_foot_score, v_total_score);
//  //         printf("-origin : %.3f,%.3f,%.3f,%.3f", v_total_score);
//         }


//         while(!continue_check)
//         {
//             idx_max = std::distance(v_total_score.begin(),max_element(v_total_score.begin(),v_total_score.end()));
//         //// exp1 different
// //            printf(" spot: (%.3f)\n",consider_length1.b);

//             // printf(" msg.array[idx_max].roughness: (%.3f)\n",consider_length1.b-msg.array[idx_max].y);
// //            if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.05+0.17<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.05+0.17)&&(0.4<abs(consider_length1.b-msg.array[idx_max].y))&&(abs(consider_length1.b-msg.array[idx_max].y)<1.0)) 
//             if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.5<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.5)) 
//             {//(0.18<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.22)&&
              
//                 msg_.a = msg.array[idx_max].x;msg_.b = msg.array[idx_max].y;
//                 msg_.c = msg.array[idx_max].z;//msg_.d = Target_Foot; 
//                 printf("msga: %.3f  b: %.3f  c: %.3f \n",msg_.a, msg_.b, msg_.c);
                
//                 continue_check = 1;
//                 if(msg.array[idx_max+31].z - msg_.c > 0.05 && msg.array[idx_max+32].z - msg_.c > 0.05 && msg.array[idx_max+33].z - msg_.c > 0.05){
//                     idx_max = idx_max-64;
//                     printf("i'm running     : 4444           min   : 222    \n");}
//                 else if (msg.array[idx_max+47].z - msg_.c > 0.05&& msg.array[idx_max+48].z - msg_.c > 0.05 && msg.array[idx_max+49].z - msg_.c > 0.05){
//                     idx_max = idx_max-48;
//                 }
//                 else if (msg.array[idx_max+63].z - msg_.c > 0.05&& msg.array[idx_max+64].z - msg_.c > 0.05 && msg.array[idx_max+65].z - msg_.c > 0.05){
//                     idx_max = idx_max-32;
//                 }
//                 else{printf("i'm not:  222(%.3f)    \n"),msg.array[idx_max+30].z;}
                
//                 // while (fabs(msg.array[idx_max].z- msg_.c)<0.05)
//                 // {
//                 //     idx_max = idx_max +16;
//                 // }
                
//                 if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
//                 idx_max = idx_max+16;
//                 }
//                 else
//                 if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
//                 idx_max = idx_max+16;
//                 }
//                 else
//                 if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
//                 idx_max = idx_max+16;
//                 }
//                 else
//                 msg_.a = msg.array[idx_max].x; msg_.b = msg.array[idx_max].y;
//                 msg_.c = msg.array[idx_max].z;
//             }
//             else
//             {
//                 v_total_score[idx_max] = 0;
//                 num++;
//                 if(num == 100) {ROS_INFO("May be no map!"); continue_check = 1;}
//                 idx_max = 256;
//             }
//             printf("idx_max: %d \n",idx_max);
//         }
//     continue_check = 0;
// }

// void submap_select(grid_map::GridMap& original_gridmap, 
//                      grid_map::GridMap& submap,
//                      const aidin_msgs::three& where_is_searching_area,
//                      const aidin_msgs::two& length)
// {
//     bool useRaw = true ;
//     bool isSuccess ;

//     grid_map::Position position(where_is_searching_area.a,where_is_searching_area.b);
//     grid_map::Length length_(length.a,length.b);
//     std::vector<std::string> layers ;//= {"elevation","variance"};
//     layers.push_back("elevation");
//     layers.push_back("variance");
//     submap = getSubmapMessage(original_gridmap, position, length_,layers,useRaw,isSuccess);
// }

// void Gridmap2pt(grid_map::GridMap& mapOut, aidin_msgs::pt_array& msg)
// {
//     aidin_msgs::pt_roughness msg_;
//     static uint32_t nPoint1 = 0 ;
//     nPoint1 = 0 ;
//     for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {
//     // Check if this is an empty cell (hole in the map).

//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
//     double x,y,z;
//     x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
    
//     if(!(-0.01<x)&&(x<0.01))
//     {
//         msg_.x = x; msg_.y = y; msg_.z = z;
    
//         msg.array.push_back(msg_);

//         nPoint1 = nPoint1 +1;
//     }
//     else;

//     }
//     msg.nPoint = nPoint1;
// }

// void gridtoimage(const grid_map_msgs::GridMap& msg, int num) //son
// {
//   ros::NodeHandle nodeHandle_;
//   std::string gridMapTopic_;
//   std::string filePath_;

//   srand((unsigned int)time(NULL));

//   filePath_="/home/son/Desktop/dataset/dataset1/elevation_map/num.png";
//   filePath_.replace(filePath_.find("num"), 3, std::to_string(num));
//   // nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/submap3"));

//   grid_map::GridMap map;
//   cv_bridge::CvImage image;
//   grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-1.0,0, image);
//   bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//   std::cout << filePath_ << std::endl;
//   sensor_msgs::Image ros_image;
//   image.toImageMsg(ros_image);
// }

// void getdataset(int _id){
//   grid_map::GridMap raw_map;
//   grid_map::GridMapRosConverter::fromMessage(Elevation_Map_Copy, raw_map);
//   gridtoimage(Elevation_Map_Copy,_id);
// }

// void SaveDataset() {
//     // create the output file stream
//     // ofstream file("/home/son/Desktop/dataset/dataset1/dataset.csv");

//     ofstream file("/home/son/Desktop/dataset/dataset1/dataset.csv", ios::app);

//     // check if the file was opened successfully
//     if (!file.is_open()) {
//         cout << "Error opening file!" << endl;
//     }

//     // write the header row
//     // file << "id,global_initial_x,global_initial_y,local_target_x,local_target_y,success_or_failure" << endl;

//     // write some sample data
//     file << id << "," << global_initial_x << "," << global_initial_y << "," << local_target_x << "," << local_target_y << "," << yaw_init << "," << yaw_target << "," << duration << "," << s_or_f << endl;


//     // close the file
//     file.close();

//     cout << "Data saved to file!" << endl;
// }

// // void subscriberCallback1 (const  grid_map_msgs::GridMap& msg1)
// // {   Elevation_Map_Copy = msg1;
// //     mappingstart = 1;}

// void msgCallbackDataset(const dataset_collector::dataset& msg) {
//     id = msg.id;
//     Elevation_Map_Copy = msg.elevation_map_raw;
//     global_initial_x = msg.global_initial_x;
//     global_initial_y = msg.global_initial_y;
//     local_target_x = msg.local_target_x;
//     local_target_y = msg.local_target_y;
//     yaw_init = msg.yaw_init;
//     yaw_target = msg.yaw_target;
//     duration = msg.duration;
//     s_or_f = msg.s_or_f;

//     std::cout << "======================================" << std::endl;
//     std::cout << "id: " << id << std::endl;
//     std::cout << "global_inital_x: " << global_initial_x << std::endl;
//     std::cout << "global_inital_y: " << global_initial_y << std::endl;
//     std::cout << "local_target_x: " << local_target_x << std::endl;
//     std::cout << "local_target_y: " << local_target_y << std::endl;;
//     std::cout << "yaw_init: " << yaw_init << std::endl;
//     std::cout << "yaw_target: " << yaw_target << std::endl;
//     std::cout << "duration(s): " << duration << std::endl;    

//     if (s_or_f == 1) {
//       std::cout << "result: success" << std::endl;  
//     }
//     else {
//       std::cout << "result: failure" << std::endl;  
//     }

//     // mappingstart = 1;
//     getdataset(id);
//     SaveDataset();
// }

// int main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "dataset");
//   ros::NodeHandle nh;
//   // ros::Subscriber sub = nh.subscribe("/aidin81/elevation_mapping/elevation_map_raw", 100, subscriberCallback1);
//   ros::Subscriber sub = nh.subscribe("/aidin81/dataset", 100, msgCallbackDataset);
//   ros::Rate loop_rate(10);

//   while (ros::ok())
//   {
//     // if(mappingstart == 1) {
//     //   getdataset();
//     // }
//     // else;
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }


// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <boost/thread.hpp>
// #include <string>
// #include <iostream>
// #include <stdio.h>
// #include <math.h>
// #include <cstdlib>
// #include <ctime>
// #include <time.h>

// // Eigen
// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

// //
// #include <sensor_msgs/image_encodings.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/imgcodecs.hpp>
// //경로주의
// //#include <opencv/cv.h>
// //#include <opencv/cv.hpp>
// // #include <opencv/highgui.h>
// // #include <opencv/highgui.hpp>

// //
// #include <nav_msgs/Odometry.h>
// #include <grid_map_ros/grid_map_ros.hpp>

// #include <grid_map_msgs/GridMap.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// // tf2
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Transform.h>

// // aidin_msgs
// #include <aidin_msgs/one.h>
// #include <aidin_msgs/two.h>
// #include <aidin_msgs/three.h>
// #include <aidin_msgs/four.h>
// #include <aidin_msgs/twel.h>
// #include <aidin_msgs/pt_array.h>
// #include <aidin_msgs/thir.h>

// #include <dataset_collector/dataset.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <grid_map_msgs/GridMap.h>
// #include <grid_map_core/GridMap.hpp>

// #define estimationRadius_normal 0.10//0.08
// #define estimationRadius_roughness  0.10//0.06//0.08//0.08//0.06
// #define criticalValue_roughness  0.05//0.05//0.01//0.007//0.02//0.02
// #define roughness_threshold 0.2

// #define ROS_RED_STREAM(SSS) ROS_INFO_STREAM("\033[31;1m" << SSS << "\033[0m")

// // ros::Publisher submap_pub_;
// // ros::Publisher pub_roughness;
// // ros::Publisher pub_height;
// // ros::Publisher normal;
// // ros::Publisher red;
// // ros::Publisher pub_foothold;
// // ros::Publisher pub_img;

// aidin_msgs::two GP;
// // const int numrow = 100;
// // const int numcol = 2;
// aidin_msgs::pt_array roughness_;
// aidin_msgs::pt_array height_;
// aidin_msgs::four footholdpredict;
// aidin_msgs::four target_pos, normal_pos;
// aidin_msgs::four temp,temp_;
// aidin_msgs::three spot;
// visualization_msgs::MarkerArray marker;
// grid_map_msgs::GridMap Elevation_Map_Copy;

// int idx_max;
// int sunseo = 0;
// int mappingstart;
// double cnt = 0;

// #include <std_msgs/Float32.h>
// #include <std_msgs/Int64.h>

// #include <fstream>
// #include <string>
// using namespace std;

// int id;
// float global_initial_x;
// float global_initial_y;
// float local_target_x;
// float local_target_y;
// float yaw_init;
// float yaw_target;
// float duration;
// bool s_or_f;
// double Z_WorldtoLocal = 0;

// double world_x = 0;
// double world_y = 0;
// double world_z = 0;

// grid_map::GridMap elevation_map_global;


// grid_map::GridMap GetLocalMap(double _x, double _y, double _z);

// void TargetFoot(const aidin_msgs::four& msg)
// {
// 		target_pos.a = msg.a;
// 		target_pos.b = msg.b;
// 		target_pos.c = msg.c;

// 		//printf("posy,posz : (%.3f, %.3f)\n",target_pos.b,target_pos.c );

// }

// void Footprint(visualization_msgs::MarkerArray *marker, aidin_msgs::four msg)
//     {
//         visualization_msgs::Marker Foot;
// 		float x, y, z;
// 		std::string str;
//         int i = 0;

// 		Foot.header.frame_id = "odom";
// 		Foot.header.stamp = ros::Time::now();


// 		Foot.type = 2;


// 		Foot.scale.x = 0.05;
// 		Foot.scale.y = 0.05;
// 		Foot.scale.z = 0.05;
		
// 		Foot.lifetime = ros::Duration();

//         Foot.ns = str;
// 		//Foot.id = i;
// 		Foot.id = 55;

// 		Foot.pose.position.x = msg.a;
// 		Foot.pose.position.y = msg.b;
// 		Foot.pose.position.z = msg.c;
// 		Foot.color.r = 1;
//         Foot.color.g = 0;//1;
//         Foot.color.b = 0;//0.5;
//         Foot.color.a = 1;

// 		Foot.id = 1;

// 		marker->markers.push_back(Foot);
 
//         Foot.lifetime = ros::Duration();

//     }

// void Footprint2(visualization_msgs::MarkerArray *marker, aidin_msgs::four msg)
//     {
//         visualization_msgs::Marker Foot;
// 		float x, y, z;
// 		std::string str;
//         int i = 0;

// 		Foot.header.frame_id = "odom";
// 		Foot.header.stamp = ros::Time::now();

// 		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// 		Foot.type = 2;

// 		// Set the scale of the marker -- 1x1x1 here means 1m on a side
// 		Foot.scale.x = 0.05;
// 		Foot.scale.y = 0.05;
// 		Foot.scale.z = 0.05;
		
// 		Foot.lifetime = ros::Duration();

//         Foot.ns = str;
// 		//Foot.id = i;
// 		Foot.id = 55;

// 		Foot.pose.position.x = msg.a;
// 		Foot.pose.position.y = msg.b;
// 		Foot.pose.position.z = msg.c;
// 		Foot.color.r = 0;
//         Foot.color.g = 1;//1;
//         Foot.color.b = 0;//0.5;
//         Foot.color.a = 1;

// 		Foot.id = 1;

// 		marker->markers.push_back(Foot);
 
//         Foot.lifetime = ros::Duration();

//     }

// void computeWithArea(grid_map::GridMap& map, 
//                                     const std::string& inputLayer, 
//                                     const std::string& outputLayersPrefix)//, Eigen::Vector2d center_) //Vector2d Position 筌�슪�숋옙�뉗퐷占쎄랜肉덌옙�숈삕占썩뫅��. 
// {
//   // ! Radius of submap for normal vector estimation.
//   double estimationRadius_ = estimationRadius_normal;
//   Eigen::Vector3d normalVectorPositiveAxis_ = grid_map::Vector3::UnitZ();
//   std::string inputLayer_ = "elevation";
//   std::string outputLayersPrefix_ = "surface_normal_";

//   for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

//     // Requested position (center) of circle in map.
//     grid_map::Position center;
//     map.getPosition(*iterator, center);
//     // Prepare data computation.
//     const int maxNumberOfCells = pow(ceil(2 * estimationRadius_ / map.getResolution()), 2);//+1筌�옙李볟㎤�우퐷占쎈챸�륅㎖占쎌컝占쎈�愿쒎㎤遺우컞筌�꼹��뜝�꾩퐷占쎄램�득쾮�ш킐冶⑤뗄李됵㎖占쎌컭冶⑨옙
//     Eigen::MatrixXd points(3, maxNumberOfCells);
//     size_t nPoint = 0;
   
//     // Gather surrounding data.
//     size_t nPoints = 0;

//     for (grid_map::CircleIterator iterator(map, center, estimationRadius_); !iterator.isPastEnd(); ++iterator) {
// //    if (!map.isValid(*iterator, inputLayer_)) continue;
//       grid_map:: Position3 point;
//       map.getPosition3(inputLayer_, *iterator, point);
//       points.col(nPoints) = point;

//       nPoints++;
//     }
//     //printf("nPoints: %.3f       ",nPoints);

//     // nPoints = nPoints + 1 ;//temp
//     if(! (nPoints <= 0)){
//       points.conservativeResize(3, nPoints); // TODO Eigen version? //筌�옙李볟㎤�곸뒇�좎럩肄좑쭪�밤럮
    
//       // Compute Eigenvectors.
//       const grid_map::Position3 mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);///////
//       const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;
    
//       const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
//       grid_map::Vector3 eigenvalues = grid_map::Vector3::Ones();
//       Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
//       // Ensure that the matrix is suited for eigenvalues calculation.
//       if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
//         const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
//         eigenvalues = solver.eigenvalues().real();
//         eigenvectors = solver.eigenvectors().real();
//       } 
//       else {
//         ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
//         // Use z-axis as default surface normal. // TODO Make dependend on surfaceNormalPositiveAxis_;
//         eigenvalues.z() = 0.0;
//       }
//       // Keep the smallest Eigenvector as normal vector.
//       int smallestId(0);
//       double smallestValue(std::numeric_limits<double>::max());
//       for (int j = 0; j < eigenvectors.cols(); j++) {
//         if (eigenvalues(j) < smallestValue) {
//           smallestId = j;
//           smallestValue = eigenvalues(j);
//         }
//       }
  
//       grid_map::Vector3 eigenvector = eigenvectors.col(smallestId);
//       if (eigenvector.dot(normalVectorPositiveAxis_) < 0.0) eigenvector = -eigenvector;
//   //    map.getIndex(center,*index);
      
//       //float normalX, normalY, normalZ;

//       //normalX = eigenvector.x();
//       //normalY = eigenvector.y();
//       //normalZ = eigenvector.z();
//       map.at(outputLayersPrefix_ + "x", *iterator) = eigenvector.x();
//       map.at(outputLayersPrefix_ + "y", *iterator) = eigenvector.y();
//       map.at(outputLayersPrefix_ + "z", *iterator) = eigenvector.z();
//       }
//       else ;
//   }
// }

// bool update_total(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut,aidin_msgs::pt_array& msg)
// {
//   unsigned char start_flag;
//   double update_time_start = ros::Time::now().toSec();
//  //ROS_INFO("total start!");
//   // Add new layers to the elevation map.
//   std::string type_ = "traversability_total";
//   mapOut = mapIn;
//   mapOut.add(type_);

//   double total,totalMax,slope,roughness,step_height;
//   double criticalValue_=0.5;//0.6

//     static uint32_t nPoint = 0 ;
//     static uint32_t nPoint_collision = 0;
//     aidin_msgs::pt_roughness msg_;



//   // First iteration through the elevation map.
//   for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
//     if (!mapOut.isValid(*iterator, "slope"))
//       continue;
//     if (!mapOut.isValid(*iterator, "traversability_roughness"))
//       continue;
// //    if (!mapOut.isValid(*iterator, "step_height"))
// //      continue;
//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
    
//     double x,y,z;
//     x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
//     msg_.x = x; msg_.y = y; msg_.z = z;

//     slope = mapOut.at("slope", *iterator);
//     roughness = mapOut.at("traversability_roughness", *iterator);
// //    step_height = mapOut.at("step_height", *iterator);

//     total = (0.5 * slope + 0.5 * roughness + 0.0 * step_height);

//     msg_.r_roughness = total;
//     //total = (0.2 * slope + 0.3*roughness + 0.0*step_height);
//     if(total < criticalValue_){mapOut.at(type_,*iterator) =  0.2+0.8*total/criticalValue_; msg_.roughness = 0.2;}//total
//     else {mapOut.at(type_,*iterator) = 1;      msg_.roughness = 1.0;}	
//     msg.array.push_back(msg_);
   

//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else; 
//     nPoint = nPoint +1;
//     if (total > totalMax) totalMax = total;
//     else ;
//   }
//     msg.nPoint = nPoint;
//     // Collision_.nPoint = nPoint_collision;

//     nPoint = 0; nPoint_collision = 0;
//   ROS_DEBUG("slope total = %f", totalMax);
//   return true;
// }

// bool update_roughness1(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)//,Eigen::Vector2d center_)
// {
//     std::string type_ = "traversability_roughness";
//     double estimationRadius_ = estimationRadius_roughness;
//     double criticalValue_ = criticalValue_roughness;

//     // Add new layer to the elevation map.
//     mapOut = mapIn;
//     mapOut.add(type_);
//     double roughnessMax = 0.0;
//     unsigned char start_flag = 0;
//     static uint32_t nPoint = 0 ;

//     for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {
    
//     // Check if this is an empty cell (hole in the map).
//     //if (!mapOut.isValid(*iterator, "surface_normal_x")) continue;
  
//     // Prepare data computation.
//     const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));//+1筌�옙李볟㎤�우퐷占쎈챸�륅㎖占쎌컝占쎈�愿쒎㎤遺우컞筌�꼹��뜝�꾩퐷占쎄램�득쾮�ш킐冶⑤뗄李됵㎖占쎌컭冶⑨옙
//     Eigen::MatrixXd points(3, maxNumberOfCells);
//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
    
//     double x,y,z;
//     x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
//     //if(!(-0.01<x)&&(x<0.01))
//     {
    
//     // Requested position (center) of circle in map.
//     grid_map::Position center;
//     center(0,0) = xyz(0,0);
//     center(1,0) = xyz(1,0);
//     //mapOut.getPosition(*iterator, center);

//     // Gather surrounding data.
//     size_t nPoints = 0;
//     double h_max = -1, h_min = 100;
//     for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
//         !submapIterator.isPastEnd(); ++submapIterator) 
//     {
//       if (!mapOut.isValid(*submapIterator, "elevation")) continue;
        
//       Eigen::Vector3d point;
//       mapOut.getPosition3("elevation", *submapIterator, point);
//       points.col(nPoints) = point;

//       h_max = std::max(h_max, point(2,0));
//       h_min = std::min(h_min, point(2,0));
//       nPoints++; 
//     }

//     if(! (nPoints <= 0)){
//     const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);
    
//     double normalX = mapOut.at("surface_normal_x", *iterator);
//     double normalY = mapOut.at("surface_normal_y", *iterator);
//     double normalZ = mapOut.at("surface_normal_z", *iterator);
//     double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
//     double sum = 0.0;
    
//     for (int i = 0; i < nPoints; i++) 
//     {
//       double dist = 1*normalX*points(0,i) + 1*normalY*points(1,i) + 1*normalZ*points(2,i) - planeParameter + 0.7*sqrt(1-normalZ*normalZ)*abs(points(2,i)-mean.z());//modifyed  0.7
//       sum += pow(dist,2);
//     }
//     double roughness = sqrt(sum / (nPoints+1 ));//-1

    
//     if (roughness < criticalValue_) 
//     {
//       mapOut.at(type_, *iterator) = 0.2+ 0.8*(roughness / criticalValue_);//1.0 - roughness / criticalValue_;

//     }
//     else 
//     {
//       mapOut.at(type_, *iterator) = 1.0;//0.0; //modify


//     }

//     //msg_.header.stamp = ros::Time::now();
//     //mapOut.at(type_, *iterator) = roughness;

//     if (roughness > roughnessMax) roughnessMax = roughness;
//     else ;
    
//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else; 

//     }
//       else ;
//     }
      
//   }

//     ROS_DEBUG("roughness max = %f", roughnessMax);
//     return true;
// }

// bool update_edge(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut,aidin_msgs::pt_array& msg)//,Eigen::Vector2d center_)
// {
//     std::string type_ = "traversability_roughness";
//     double estimationRadius_ = estimationRadius_roughness;
//     double criticalValue_ = criticalValue_roughness;

//     mapOut = mapIn;
//     mapOut.add(type_);
//     double roughnessMax = 0.0;
//     unsigned char start_flag = 0;

//     static uint32_t nPoint = 0 ;
//     aidin_msgs::pt_roughness msg_;
//     for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {
//     const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));//+1해주었었음
//     Eigen::MatrixXd points(3, maxNumberOfCells);
//     //3 행 maxNumberOfCells 렬의 행렬. 
//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
//       double x,y,z;
//       x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
// //      printf("z:%f\n",z);
//       //if(!(-0.01<x)&&(x<0.01))
//       {msg_.x = x; msg_.y = y; msg_.z = z;

//     grid_map::Position center;
//     center(0,0) = xyz(0,0);
//     center(1,0) = xyz(1,0);
//     //mapOut.getPosition(*iterator, center);

//     // Gather surrounding data.
//     size_t nPoints = 0;
//     double h_max = -1, h_min = 100;
//     for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
//         !submapIterator.isPastEnd(); ++submapIterator) 
//     {
//       if (!mapOut.isValid(*submapIterator, "elevation")) continue;
        
//       Eigen::Vector3d point;
//       mapOut.getPosition3("elevation", *submapIterator, point);
//       points.col(nPoints) = point;

//       h_max = std::max(h_max, point(2,0));
//       h_min = std::min(h_min, point(2,0));


//       nPoints++; 


//     }

//     if(! (nPoints <= 0)){
//     const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);
    
//     double sum = 0.0;
//     for (int i = 0; i < nPoints; i++) 
//     {
//       double dist = fabs(mean.z()-points(2,nPoints));;
//       sum += dist;
//     }
//     double roughness = h_max - h_min;    
// 	//ROS_INFO("roughness: %5f\n",roughness);     
//     msg_.r_roughness = roughness;
//     if (roughness < criticalValue_) //modify
//     {
//       mapOut.at(type_, *iterator) = 0.2;//1.0 - roughness / criticalValue_;
//       msg_.roughness = 0.2; 
// //      printf("ture\n");
//     }
//     else
//     {
//       mapOut.at(type_, *iterator) = 1.0;//0.0; //modify
//       msg_.roughness = 1.0;
//       printf("false\n");
//     }
//     //msg_.header.stamp = ros::Time::now();
//     //mapOut.at(type_, *iterator) = roughness;
//     msg.array.push_back(msg_);
//     if (roughness > roughnessMax) roughnessMax = roughness;
//     else ;

//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else; 
//     nPoint = nPoint +1;
//       }
//       else ;
//       }
      
//   }

//     msg.nPoint = nPoint;
//     nPoint = 0;
//     ROS_DEBUG("roughness max = %f", roughnessMax);
//     return true;
// }

// // void update_heightedge(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
// // {       
// //     ROS_INFO("normal Vector Start!");
// //         if(update_edge(submap,submap,msg)) ROS_RED_STREAM("Complete update_heightedge in Enable_Traverse");
// //         else ROS_WARN_STREAM("Not Complete update_traverse in Enable_Traverse");
// // }

// bool update_slope(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)
// {
//   double update_time_start = ros::Time::now().toSec();

//   std::string type_ = "slope";
//   unsigned char start_flag=0;
//   // Add new layer to the elevation map.
//   mapOut = mapIn;
//   mapOut.add(type_);

//   double slope, slopeMax = 0.0,slopeMin = 0.5;
//   double criticalValue_ = 0.78;
  
//   for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {

//     // Check if there is a surface normal (empty cell).
//     if (!mapOut.isValid(*iterator, "surface_normal_z")) continue;

//     // Compute slope from surface normal z
//     slope = acos(mapOut.at("surface_normal_z", *iterator));

//     if (slope < criticalValue_) {
//       mapOut.at(type_, *iterator) = 0.2 + 0.8*(slope/criticalValue_);//1.0 - slope / criticalValue_;
//     }
//     else {
//       mapOut.at(type_, *iterator) = 1.0;//0.0;
//     }

//     if (slope > slopeMax) slopeMax = slope;
//     if (slope < slopeMin) slopeMin = slope;
    
//     if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
//     else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
//     else;


//   }
//   //ROS_INFO("Slope_MIN/MAX :%.3f, %.3f",slopeMax,slopeMin);
//   ROS_DEBUG("slope max = %f", slopeMax);

//   return true;
// }

// bool update_normalvector(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)//, Eigen::Vector2d center_) 
// {
//   //! Normal vector positive axis.
//   Eigen::Vector3d normalVectorPositiveAxis_ = grid_map::Vector3::UnitZ();

//   //! Input layer name.
//   std::string inputLayer_ = "elevation";

//   //! Output layer name.
//   std::string outputLayersPrefix_ = "surface_normal_";
 
//   //ROS_INFO_THROTTLE(0.3, "update Normal_vector!");
//   std::vector<std::string> normalVectorsLayers;
//   normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
//   normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
//   normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

//   mapOut = mapIn;
//   for (const auto& layer : normalVectorsLayers) mapOut.add(layer);
 
//     computeWithArea(mapOut, inputLayer_, outputLayersPrefix_);

//   return true;
// }

// void update_traverse(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
// {       
//     if(update_normalvector(submap,submap))
//     {
// 	if(update_slope(submap,submap)){if(update_roughness1(submap,submap)) {if(update_total(submap,submap,msg))ROS_RED_STREAM("Complete update_traverse in Enable_Traverse");}}
//         else ;
//     }
//     else ; 
// }

// grid_map::GridMap getSubmapMessage(grid_map::GridMap& original_gridmap, //const grid_map::GridMap& original_gridmap, 
//                                     const grid_map::Position& position,
//                                     const grid_map::Length& length,
//                                     const std::vector<std::string>& layers,
//                                     const bool useRaw, bool& isSuccess)
// {
//     grid_map::Index index;
  
//     grid_map::GridMap subMap;
//     if (useRaw) 
//     {   
//         subMap = original_gridmap.getSubmap(position, length, index, isSuccess);
//     } else ROS_INFO("useRaw is false.");
//     grid_map_msgs::GridMap message;
//     grid_map::GridMapRosConverter::toMessage(subMap, layers, message);

//     return subMap; 
// }

// void foothold_detect(const aidin_msgs::pt_array& msg,
//                                     const aidin_msgs::three& G_P_F, 
//                                     aidin_msgs::four& msg_)
// {
//     //ROS_INFO("come in foothold");
//     int nPoint_ = (msg.nPoint); // array 갯수
//     unsigned char continue_check = 0,num = 0;
//     float x = G_P_F.a;
//     float y = G_P_F.b;
//     float z = G_P_F.c;
//     //msg_.d = G_P_F.d;
//     //float dist ;
//     std::vector<double> v_length_dist, v_origin_dist, v_foot_dist; 
//     std::vector<double> v_length_score, v_origin_score, v_foot_score, v_total_score; 
//     Eigen::Vector3d G_P_ss_;
//     G_P_ss_(0) = G_P_F.a;
//     G_P_ss_(1) = G_P_F.b;
//     G_P_ss_(2) = G_P_F.c;
//     // if(nPoint_ < 5) 
//     // {
//     //     printf("- Submap Not Complete :  nPoint_ : %d !!!!!!\n", nPoint_);
//     //     msg_.a = x; msg_.b = y; msg_.c = 0; msg_.d = FailMapData;

//     // }
//     // else
//     // {
//         for(int i = 0; i < nPoint_; i++)
//         {
//             float dist1, dist2, dist3;
//             //if((-0.05<msg.array[i].z)&&(msg.array[i].z<0.05))
//             {
//             dist1 = sqrt(pow(msg.array[i].x - G_P_ss_(0), 2) + pow(msg.array[i].y - G_P_ss_(1), 2) + pow(msg.array[i].z - G_P_ss_(2), 2));;
//             v_origin_dist.push_back(dist1);
//             dist2 = sqrt(pow(msg.array[i].x - G_P_ss_(0), 2) + pow(msg.array[i].y - G_P_ss_(1), 2));
//             v_length_dist.push_back(dist2);
//             dist3 = sqrt((msg.array[i].x-x)*(msg.array[i].x-x)
//                     +(msg.array[i].y-y)*(msg.array[i].y-y));
//             v_foot_dist.push_back(dist3);
//             }
//         }
//         double origin_max = *max_element(v_origin_dist.begin(),v_origin_dist.end());
//         double origin_min = *min_element(v_origin_dist.begin(),v_origin_dist.end());
//         printf("-origin_max     : %.3f           min   : %.3f       \n",origin_max , origin_min);
//         double length_max = *max_element(v_length_dist.begin(),v_length_dist.end());
//         double length_min = *min_element(v_length_dist.begin(),v_length_dist.end());

//         double foot_max = *max_element(v_foot_dist.begin(),v_foot_dist.end());
//         double foot_min = *min_element(v_foot_dist.begin(),v_foot_dist.end());
        
//         for(int i = 0; i < nPoint_; i++)
//         {  
//           //v_origin_score.push_back( ( origin_max ) - ( v_origin_dist[i] ) );
//           //v_length_score.push_back( ( v_length_dist[i] ) - (length_max ) ); //max length
//           //v_foot_score.push_back( (foot_max ) - ( v_foot_dist[i] / (foot_max-foot_min)) );
//           //v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + v_foot_score[i] );
        
//           // 나누기는 여기밖에......
//           v_origin_score.push_back( ( origin_max / (origin_max-origin_min)) - ( v_origin_dist[i] / (origin_max-origin_min)) );
//           v_length_score.push_back( ( v_length_dist[i] / (length_max-length_min)) - (length_max / (length_max-length_min)) ); //max length
//           v_foot_score.push_back( (foot_max / (foot_max-foot_min)) - ( v_foot_dist[i] / (foot_max-foot_min)) );
//           v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + 2*v_foot_score[i] );
// //          std::cout <<"origin"<<v_origin_score<<"length"<<v_length_score;
// //        printf("-origin : %.3f,%.3f,%.3f,%.3f    length : %.3f,%.3f,%.3f,%.3f   foot : %.3f   total : %.3f \n",v_origin_score, v_length_score, v_foot_score, v_total_score);
//  //         printf("-origin : %.3f,%.3f,%.3f,%.3f", v_total_score);
//         }


//         while(!continue_check)
//         {
//             idx_max = std::distance(v_total_score.begin(),max_element(v_total_score.begin(),v_total_score.end()));
//         //// exp1 different
// //            printf(" spot: (%.3f)\n",consider_length1.b);

//             // printf(" msg.array[idx_max].roughness: (%.3f)\n",consider_length1.b-msg.array[idx_max].y);
// //            if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.05+0.17<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.05+0.17)&&(0.4<abs(consider_length1.b-msg.array[idx_max].y))&&(abs(consider_length1.b-msg.array[idx_max].y)<1.0)) 
//             if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.5<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.5)) 
//             {//(0.18<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.22)&&
              
//                 msg_.a = msg.array[idx_max].x;msg_.b = msg.array[idx_max].y;
//                 msg_.c = msg.array[idx_max].z;//msg_.d = Target_Foot; 
//                 printf("msga: %.3f  b: %.3f  c: %.3f \n",msg_.a, msg_.b, msg_.c);
                
//                 continue_check = 1;
//                 if(msg.array[idx_max+31].z - msg_.c > 0.05 && msg.array[idx_max+32].z - msg_.c > 0.05 && msg.array[idx_max+33].z - msg_.c > 0.05){
//                     idx_max = idx_max-64;
//                     printf("i'm running     : 4444           min   : 222    \n");}
//                 else if (msg.array[idx_max+47].z - msg_.c > 0.05&& msg.array[idx_max+48].z - msg_.c > 0.05 && msg.array[idx_max+49].z - msg_.c > 0.05){
//                     idx_max = idx_max-48;
//                 }
//                 else if (msg.array[idx_max+63].z - msg_.c > 0.05&& msg.array[idx_max+64].z - msg_.c > 0.05 && msg.array[idx_max+65].z - msg_.c > 0.05){
//                     idx_max = idx_max-32;
//                 }
//                 else{printf("i'm not:  222(%.3f)    \n"),msg.array[idx_max+30].z;}
                
//                 // while (fabs(msg.array[idx_max].z- msg_.c)<0.05)
//                 // {
//                 //     idx_max = idx_max +16;
//                 // }
                
//                 if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
//                 idx_max = idx_max+16;
//                 }
//                 else
//                 if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
//                 idx_max = idx_max+16;
//                 }
//                 else
//                 if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
//                 idx_max = idx_max+16;
//                 }
//                 else
//                 msg_.a = msg.array[idx_max].x; msg_.b = msg.array[idx_max].y;
//                 msg_.c = msg.array[idx_max].z;
//             }
//             else
//             {
//                 v_total_score[idx_max] = 0;
//                 num++;
//                 if(num == 100) {ROS_INFO("May be no map!"); continue_check = 1;}
//                 idx_max = 256;
//             }
//             printf("idx_max: %d \n",idx_max);
//         }
//     continue_check = 0;
// }

// void submap_select(grid_map::GridMap& original_gridmap, 
//                      grid_map::GridMap& submap,
//                      const aidin_msgs::three& where_is_searching_area,
//                      const aidin_msgs::two& length)
// {
//     bool useRaw = true ;
//     bool isSuccess ;

//     grid_map::Position position(where_is_searching_area.a,where_is_searching_area.b);
//     grid_map::Length length_(length.a,length.b);
//     std::vector<std::string> layers ;//= {"elevation","variance"};
//     layers.push_back("elevation");
//     layers.push_back("variance");
//     submap = getSubmapMessage(original_gridmap, position, length_,layers,useRaw,isSuccess);
// }

// void Gridmap2pt(grid_map::GridMap& mapOut, aidin_msgs::pt_array& msg)
// {
//     aidin_msgs::pt_roughness msg_;
//     static uint32_t nPoint1 = 0 ;
//     nPoint1 = 0 ;
//     for (grid_map::GridMapIterator iterator(mapOut);
//       !iterator.isPastEnd(); ++iterator) {
//     // Check if this is an empty cell (hole in the map).

//     Eigen::Vector3d xyz;
//     mapOut.getPosition3("elevation", *iterator, xyz);
//     double x,y,z;
//     x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
    
//     if(!(-0.01<x)&&(x<0.01))
//     {
//         msg_.x = x; msg_.y = y; msg_.z = z;
    
//         msg.array.push_back(msg_);

//         nPoint1 = nPoint1 +1;
//     }
//     else;

//     }
//     msg.nPoint = nPoint1;
// }

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
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-0.5,1.5, image);
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
//   grid_map::GridMapRosConverter::toCvImage(map,"elevation_inpainted", sensor_msgs::image_encodings::MONO8,-0.5,1.5, image);
//   bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//   std::cout << filePath_ << std::endl;
//   sensor_msgs::Image ros_image;
//   image.toImageMsg(ros_image);
// }

// // void getdataset(int _id){
// //   // grid_map::GridMap raw_map;
// //   // grid_map::GridMapRosConverter::fromMessage(Elevation_Map_Copy, raw_map);
// //   gridtoimage_pre(Elevation_Map_Copy, _id); // previous method
// //   gridtoimage_cur(elevation_map_local, _id); // current method
// // }

// void SaveDataset() {
//     // create the output file stream
//     // ofstream file("/home/son/Desktop/dataset/dataset1/dataset.csv");

//     // ofstream file("/home/son/Desktop/dataset/dataset1/dataset.csv", ios::app);

//     // // check if the file was opened successfully
//     // if (!file.is_open()) {
//     //     cout << "Error opening file!" << endl;
//     // }

//     // // write the header row
//     // // file << "id,global_initial_x,global_initial_y,local_target_x,local_target_y,success_or_failure" << endl;

//     // // write some sample data
//     // file << id << "," << global_initial_x << "," << global_initial_y << "," << local_target_x << "," << local_target_y << "," << yaw_init << "," << yaw_target << "," << duration << "," << s_or_f << endl;


//     // // close the file
//     // file.close();

//     // cout << "Data saved to file!" << endl;
// }

// // void subscriberCallback1 (const  grid_map_msgs::GridMap& msg1)
// // {   Elevation_Map_Copy = msg1;
// //     mappingstart = 1;}

// void msgCallbackDataset(const dataset_collector::dataset& msg) {
//     id = msg.id;
//     Elevation_Map_Copy = msg.elevation_map_raw;
//     world_x = msg.world_x_init;
//     world_y = msg.world_y_init;
//     world_z = msg.world_z_init; // add 
//     // local_target_x = msg.local_target_x;
//     // local_target_y = msg.local_target_y;
//     // yaw_init = msg.yaw_init;
//     // yaw_target = msg.yaw_target;
//     // duration = msg.duration;
//     s_or_f = msg.s_or_f;
    

//     std::cout << "======================================" << std::endl;
//     std::cout << "id: " << id << std::endl;
//     std::cout << "world_z: " << global_initial_x << std::endl;
//     std::cout << "world_y: " << global_initial_y << std::endl;
//     std::cout << "world_z: " << local_target_x << std::endl;
//     // std::cout << "local_target_y: " << local_target_y << std::endl;;
//     // std::cout << "yaw_init: " << yaw_init << std::endl;
//     // std::cout << "yaw_target: " << yaw_target << std::endl;
//     // std::cout << "duration(s): " << duration << std::endl;    

//     if (s_or_f == 1) {
//       std::cout << "result: success" << std::endl;  
//     }
//     else {
//       std::cout << "result: failure" << std::endl;  
//     }

//     // SaveDataset();

// //     // mappingstart = 1;

//     // grid_map::GridMap elevation_map_local_pre;
//     // grid_map::GridMapRosConverter::fromMessage(Elevation_Map_Copy, elevation_map_local_pre);

//     gridtoimage_pre(Elevation_Map_Copy, id); // previous method

//     // grid_map::GridMap elevation_map_local;
//     grid_map_msgs::GridMap elevation_map_local_msgs
//     elevation_map_local = GetLocalMap(world_x, world_y, world_z);
//     // grid_map::GridMapRosConverter::toMessage(elevation_map_local, "elevation_inpainted", elevation_map_local_msgs);

//     gridtoimage_cur(elevation_map_local_msgs, id); // current method

// }

// // void msgCallbackZ(const std_msgs::Float64MultiArray& msg) {
// //     Z_WorldtoLocal = msg->data[2];
// // }

// void GetGlobalMap() {
//   // Open the bag file
//   rosbag::Bag bag;
//   bag.open("home/son/Desktop/dataset/dataset1/global_map_1.bag", rosbag::bagmode::Read);

//   // Create a view for the desired topic
//   rosbag::View view(bag, rosbag::TopicQuery("/elevation_mapping/elevation_map_raw"));

//   // Iterate through the messages in the view
//   for (const rosbag::MessageInstance& m : view)
//   {
//     // Extract the grid_map_msgs/GridMap message
//     grid_map_msgs::GridMap::ConstPtr grid_map_msg = m.instantiate<grid_map_msgs::GridMap>();
//     if (grid_map_msg != nullptr)
//     {
//       // Convert the message to a GridMap object
//       // grid_map::GridMap elevation_map_global;
//       grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, elevation_map_global);
//     }
//   }

//   // Close the bag file
//   bag.close();  
// }

// grid_map::GridMap GetLocalMap(double _x, double _y, double _z) {

//   // Define submap center and size
//   grid_map::Position submap_center(_x, _y);  // center of submap in map coordinates
//   double submap_size = 2.5;  // size of submap in meters

//   // Extract submap
//   grid_map::GridMap _elevation_map_local;
//   elevation_map_global.getSubmap(submap_center, submap_size, _elevation_map_local);

//   // Add 0.8 to submap data
//   _elevation_map_local.add("elevation_inpainted", _z);

//   return _elevation_map_local;
// }

// int main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "dataset_collector");
//   ros::NodeHandle nh;
//   // ros::Subscriber sub = nh.subscribe("/aidin81/elevation_mapping/elevation_map_raw", 100, subscriberCallback1);
//   ros::Subscriber sub_dataset = nh.subscribe("/aidin81/dataset", 100, msgCallbackDataset);
//   // ros::Subscriber sub_z = nh.subscribe("/aidin81/BodyPos_sim", 100, msgCallbackZ);
  
//   GetGlobalMap(); // get grid_map::GridMap elevation_map_global

//   ros::Rate loop_rate(10);

//   while (ros::ok())
//   {
//     // if(mappingstart == 1) {
//     //   getdataset();
//     // }
//     // else;
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }



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

// aidin_msgs
#include <aidin_msgs/one.h>
#include <aidin_msgs/two.h>
#include <aidin_msgs/three.h>
#include <aidin_msgs/four.h>
#include <aidin_msgs/twel.h>
#include <aidin_msgs/pt_array.h>
#include <aidin_msgs/thir.h>

#include <dataset_collector/dataset.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

#define estimationRadius_normal 0.10//0.08
#define estimationRadius_roughness  0.10//0.06//0.08//0.08//0.06
#define criticalValue_roughness  0.05//0.05//0.01//0.007//0.02//0.02
#define roughness_threshold 0.2

#define ROS_RED_STREAM(SSS) ROS_INFO_STREAM("\033[31;1m" << SSS << "\033[0m")

// ros::Publisher submap_pub_;
// ros::Publisher pub_roughness;
// ros::Publisher pub_height;
// ros::Publisher normal;
// ros::Publisher red;
// ros::Publisher pub_foothold;
// ros::Publisher pub_img;

aidin_msgs::two GP;
// const int numrow = 100;
// const int numcol = 2;
aidin_msgs::pt_array roughness_;
aidin_msgs::pt_array height_;
aidin_msgs::four footholdpredict;
aidin_msgs::four target_pos, normal_pos;
aidin_msgs::four temp,temp_;
aidin_msgs::three spot;
visualization_msgs::MarkerArray marker;
grid_map_msgs::GridMap Elevation_Map_Copy;

int idx_max;
int sunseo = 0;
int mappingstart;
double cnt = 0;

#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

#include <fstream>
#include <string>
using namespace std;

int id;
float global_initial_x;
float global_initial_y;
float local_target_x;
float local_target_y;
float yaw_init;
float yaw_target;
float duration;
bool s_or_f;
double Z_WorldtoLocal = 0;

double world_x_init = 0;
double world_y_init = 0;
double world_z_init = 0;

grid_map::GridMap elevation_map_global;

grid_map::GridMap GetLocalMap(double _world_x_init, double _world_y_init, double _world_z_init);

void TargetFoot(const aidin_msgs::four& msg)
{
		target_pos.a = msg.a;
		target_pos.b = msg.b;
		target_pos.c = msg.c;

		//printf("posy,posz : (%.3f, %.3f)\n",target_pos.b,target_pos.c );

}

void Footprint(visualization_msgs::MarkerArray *marker, aidin_msgs::four msg)
    {
        visualization_msgs::Marker Foot;
		float x, y, z;
		std::string str;
        int i = 0;

		Foot.header.frame_id = "odom";
		Foot.header.stamp = ros::Time::now();


		Foot.type = 2;


		Foot.scale.x = 0.05;
		Foot.scale.y = 0.05;
		Foot.scale.z = 0.05;
		
		Foot.lifetime = ros::Duration();

        Foot.ns = str;
		//Foot.id = i;
		Foot.id = 55;

		Foot.pose.position.x = msg.a;
		Foot.pose.position.y = msg.b;
		Foot.pose.position.z = msg.c;
		Foot.color.r = 1;
        Foot.color.g = 0;//1;
        Foot.color.b = 0;//0.5;
        Foot.color.a = 1;

		Foot.id = 1;

		marker->markers.push_back(Foot);
 
        Foot.lifetime = ros::Duration();

    }

void Footprint2(visualization_msgs::MarkerArray *marker, aidin_msgs::four msg)
    {
        visualization_msgs::Marker Foot;
		float x, y, z;
		std::string str;
        int i = 0;

		Foot.header.frame_id = "odom";
		Foot.header.stamp = ros::Time::now();

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		Foot.type = 2;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		Foot.scale.x = 0.05;
		Foot.scale.y = 0.05;
		Foot.scale.z = 0.05;
		
		Foot.lifetime = ros::Duration();

        Foot.ns = str;
		//Foot.id = i;
		Foot.id = 55;

		Foot.pose.position.x = msg.a;
		Foot.pose.position.y = msg.b;
		Foot.pose.position.z = msg.c;
		Foot.color.r = 0;
        Foot.color.g = 1;//1;
        Foot.color.b = 0;//0.5;
        Foot.color.a = 1;

		Foot.id = 1;

		marker->markers.push_back(Foot);
 
        Foot.lifetime = ros::Duration();

    }

void computeWithArea(grid_map::GridMap& map, 
                                    const std::string& inputLayer, 
                                    const std::string& outputLayersPrefix)//, Eigen::Vector2d center_) //Vector2d Position 筌�슪�숋옙�뉗퐷占쎄랜肉덌옙�숈삕占썩뫅��. 
{
  // ! Radius of submap for normal vector estimation.
  double estimationRadius_ = estimationRadius_normal;
  Eigen::Vector3d normalVectorPositiveAxis_ = grid_map::Vector3::UnitZ();
  std::string inputLayer_ = "elevation";
  std::string outputLayersPrefix_ = "surface_normal_";

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

    // Requested position (center) of circle in map.
    grid_map::Position center;
    map.getPosition(*iterator, center);
    // Prepare data computation.
    const int maxNumberOfCells = pow(ceil(2 * estimationRadius_ / map.getResolution()), 2);//+1筌�옙李볟㎤�우퐷占쎈챸�륅㎖占쎌컝占쎈�愿쒎㎤遺우컞筌�꼹��뜝�꾩퐷占쎄램�득쾮�ш킐冶⑤뗄李됵㎖占쎌컭冶⑨옙
    Eigen::MatrixXd points(3, maxNumberOfCells);
    size_t nPoint = 0;
   
    // Gather surrounding data.
    size_t nPoints = 0;

    for (grid_map::CircleIterator iterator(map, center, estimationRadius_); !iterator.isPastEnd(); ++iterator) {
//    if (!map.isValid(*iterator, inputLayer_)) continue;
      grid_map:: Position3 point;
      map.getPosition3(inputLayer_, *iterator, point);
      points.col(nPoints) = point;

      nPoints++;
    }
    //printf("nPoints: %.3f       ",nPoints);

    // nPoints = nPoints + 1 ;//temp
    if(! (nPoints <= 0)){
      points.conservativeResize(3, nPoints); // TODO Eigen version? //筌�옙李볟㎤�곸뒇�좎럩肄좑쭪�밤럮
    
      // Compute Eigenvectors.
      const grid_map::Position3 mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);///////
      const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;
    
      const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
      grid_map::Vector3 eigenvalues = grid_map::Vector3::Ones();
      Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
      // Ensure that the matrix is suited for eigenvalues calculation.
      if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
        const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
        eigenvalues = solver.eigenvalues().real();
        eigenvectors = solver.eigenvectors().real();
      } 
      else {
        ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
        // Use z-axis as default surface normal. // TODO Make dependend on surfaceNormalPositiveAxis_;
        eigenvalues.z() = 0.0;
      }
      // Keep the smallest Eigenvector as normal vector.
      int smallestId(0);
      double smallestValue(std::numeric_limits<double>::max());
      for (int j = 0; j < eigenvectors.cols(); j++) {
        if (eigenvalues(j) < smallestValue) {
          smallestId = j;
          smallestValue = eigenvalues(j);
        }
      }
  
      grid_map::Vector3 eigenvector = eigenvectors.col(smallestId);
      if (eigenvector.dot(normalVectorPositiveAxis_) < 0.0) eigenvector = -eigenvector;
  //    map.getIndex(center,*index);
      
      //float normalX, normalY, normalZ;

      //normalX = eigenvector.x();
      //normalY = eigenvector.y();
      //normalZ = eigenvector.z();
      map.at(outputLayersPrefix_ + "x", *iterator) = eigenvector.x();
      map.at(outputLayersPrefix_ + "y", *iterator) = eigenvector.y();
      map.at(outputLayersPrefix_ + "z", *iterator) = eigenvector.z();
      }
      else ;
  }
}

bool update_total(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut,aidin_msgs::pt_array& msg)
{
  unsigned char start_flag;
  double update_time_start = ros::Time::now().toSec();
 //ROS_INFO("total start!");
  // Add new layers to the elevation map.
  std::string type_ = "traversability_total";
  mapOut = mapIn;
  mapOut.add(type_);

  double total,totalMax,slope,roughness,step_height;
  double criticalValue_=0.5;//0.6

    static uint32_t nPoint = 0 ;
    static uint32_t nPoint_collision = 0;
    aidin_msgs::pt_roughness msg_;



  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, "slope"))
      continue;
    if (!mapOut.isValid(*iterator, "traversability_roughness"))
      continue;
//    if (!mapOut.isValid(*iterator, "step_height"))
//      continue;
    Eigen::Vector3d xyz;
    mapOut.getPosition3("elevation", *iterator, xyz);
    
    double x,y,z;
    x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
    msg_.x = x; msg_.y = y; msg_.z = z;

    slope = mapOut.at("slope", *iterator);
    roughness = mapOut.at("traversability_roughness", *iterator);
//    step_height = mapOut.at("step_height", *iterator);

    total = (0.5 * slope + 0.5 * roughness + 0.0 * step_height);

    msg_.r_roughness = total;
    //total = (0.2 * slope + 0.3*roughness + 0.0*step_height);
    if(total < criticalValue_){mapOut.at(type_,*iterator) =  0.2+0.8*total/criticalValue_; msg_.roughness = 0.2;}//total
    else {mapOut.at(type_,*iterator) = 1;      msg_.roughness = 1.0;}	
    msg.array.push_back(msg_);
   

    if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
    else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
    else; 
    nPoint = nPoint +1;
    if (total > totalMax) totalMax = total;
    else ;
  }
    msg.nPoint = nPoint;
    // Collision_.nPoint = nPoint_collision;

    nPoint = 0; nPoint_collision = 0;
  ROS_DEBUG("slope total = %f", totalMax);
  return true;
}

bool update_roughness1(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)//,Eigen::Vector2d center_)
{
    std::string type_ = "traversability_roughness";
    double estimationRadius_ = estimationRadius_roughness;
    double criticalValue_ = criticalValue_roughness;

    // Add new layer to the elevation map.
    mapOut = mapIn;
    mapOut.add(type_);
    double roughnessMax = 0.0;
    unsigned char start_flag = 0;
    static uint32_t nPoint = 0 ;

    for (grid_map::GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {
    
    // Check if this is an empty cell (hole in the map).
    //if (!mapOut.isValid(*iterator, "surface_normal_x")) continue;
  
    // Prepare data computation.
    const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));//+1筌�옙李볟㎤�우퐷占쎈챸�륅㎖占쎌컝占쎈�愿쒎㎤遺우컞筌�꼹��뜝�꾩퐷占쎄램�득쾮�ш킐冶⑤뗄李됵㎖占쎌컭冶⑨옙
    Eigen::MatrixXd points(3, maxNumberOfCells);
    Eigen::Vector3d xyz;
    mapOut.getPosition3("elevation", *iterator, xyz);
    
    double x,y,z;
    x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
    //if(!(-0.01<x)&&(x<0.01))
    {
    
    // Requested position (center) of circle in map.
    grid_map::Position center;
    center(0,0) = xyz(0,0);
    center(1,0) = xyz(1,0);
    //mapOut.getPosition(*iterator, center);

    // Gather surrounding data.
    size_t nPoints = 0;
    double h_max = -1, h_min = 100;
    for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
        !submapIterator.isPastEnd(); ++submapIterator) 
    {
      if (!mapOut.isValid(*submapIterator, "elevation")) continue;
        
      Eigen::Vector3d point;
      mapOut.getPosition3("elevation", *submapIterator, point);
      points.col(nPoints) = point;

      h_max = std::max(h_max, point(2,0));
      h_min = std::min(h_min, point(2,0));
      nPoints++; 
    }

    if(! (nPoints <= 0)){
    const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);
    
    double normalX = mapOut.at("surface_normal_x", *iterator);
    double normalY = mapOut.at("surface_normal_y", *iterator);
    double normalZ = mapOut.at("surface_normal_z", *iterator);
    double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
    double sum = 0.0;
    
    for (int i = 0; i < nPoints; i++) 
    {
      double dist = 1*normalX*points(0,i) + 1*normalY*points(1,i) + 1*normalZ*points(2,i) - planeParameter + 0.7*sqrt(1-normalZ*normalZ)*abs(points(2,i)-mean.z());//modifyed  0.7
      sum += pow(dist,2);
    }
    double roughness = sqrt(sum / (nPoints+1 ));//-1

    
    if (roughness < criticalValue_) 
    {
      mapOut.at(type_, *iterator) = 0.2+ 0.8*(roughness / criticalValue_);//1.0 - roughness / criticalValue_;

    }
    else 
    {
      mapOut.at(type_, *iterator) = 1.0;//0.0; //modify


    }

    //msg_.header.stamp = ros::Time::now();
    //mapOut.at(type_, *iterator) = roughness;

    if (roughness > roughnessMax) roughnessMax = roughness;
    else ;
    
    if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
    else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
    else; 

    }
      else ;
    }
      
  }

    ROS_DEBUG("roughness max = %f", roughnessMax);
    return true;
}

bool update_edge(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut,aidin_msgs::pt_array& msg)//,Eigen::Vector2d center_)
{
    std::string type_ = "traversability_roughness";
    double estimationRadius_ = estimationRadius_roughness;
    double criticalValue_ = criticalValue_roughness;

    mapOut = mapIn;
    mapOut.add(type_);
    double roughnessMax = 0.0;
    unsigned char start_flag = 0;

    static uint32_t nPoint = 0 ;
    aidin_msgs::pt_roughness msg_;
    for (grid_map::GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {
    const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));//+1해주었었음
    Eigen::MatrixXd points(3, maxNumberOfCells);
    //3 행 maxNumberOfCells 렬의 행렬. 
    Eigen::Vector3d xyz;
    mapOut.getPosition3("elevation", *iterator, xyz);
      double x,y,z;
      x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
//      printf("z:%f\n",z);
      //if(!(-0.01<x)&&(x<0.01))
      {msg_.x = x; msg_.y = y; msg_.z = z;

    grid_map::Position center;
    center(0,0) = xyz(0,0);
    center(1,0) = xyz(1,0);
    //mapOut.getPosition(*iterator, center);

    // Gather surrounding data.
    size_t nPoints = 0;
    double h_max = -1, h_min = 100;
    for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
        !submapIterator.isPastEnd(); ++submapIterator) 
    {
      if (!mapOut.isValid(*submapIterator, "elevation")) continue;
        
      Eigen::Vector3d point;
      mapOut.getPosition3("elevation", *submapIterator, point);
      points.col(nPoints) = point;

      h_max = std::max(h_max, point(2,0));
      h_min = std::min(h_min, point(2,0));


      nPoints++; 


    }

    if(! (nPoints <= 0)){
    const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / (nPoints+1);
    
    double sum = 0.0;
    for (int i = 0; i < nPoints; i++) 
    {
      double dist = fabs(mean.z()-points(2,nPoints));;
      sum += dist;
    }
    double roughness = h_max - h_min;    
	//ROS_INFO("roughness: %5f\n",roughness);     
    msg_.r_roughness = roughness;
    if (roughness < criticalValue_) //modify
    {
      mapOut.at(type_, *iterator) = 0.2;//1.0 - roughness / criticalValue_;
      msg_.roughness = 0.2; 
//      printf("ture\n");
    }
    else
    {
      mapOut.at(type_, *iterator) = 1.0;//0.0; //modify
      msg_.roughness = 1.0;
      printf("false\n");
    }
    //msg_.header.stamp = ros::Time::now();
    //mapOut.at(type_, *iterator) = roughness;
    msg.array.push_back(msg_);
    if (roughness > roughnessMax) roughnessMax = roughness;
    else ;

    if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
    else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
    else; 
    nPoint = nPoint +1;
      }
      else ;
      }
      
  }

    msg.nPoint = nPoint;
    nPoint = 0;
    ROS_DEBUG("roughness max = %f", roughnessMax);
    return true;
}

// void update_heightedge(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
// {       
//     ROS_INFO("normal Vector Start!");
//         if(update_edge(submap,submap,msg)) ROS_RED_STREAM("Complete update_heightedge in Enable_Traverse");
//         else ROS_WARN_STREAM("Not Complete update_traverse in Enable_Traverse");
// }

bool update_slope(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)
{
  double update_time_start = ros::Time::now().toSec();

  std::string type_ = "slope";
  unsigned char start_flag=0;
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double slope, slopeMax = 0.0,slopeMin = 0.5;
  double criticalValue_ = 0.78;
  
  for (grid_map::GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {

    // Check if there is a surface normal (empty cell).
    if (!mapOut.isValid(*iterator, "surface_normal_z")) continue;

    // Compute slope from surface normal z
    slope = acos(mapOut.at("surface_normal_z", *iterator));

    if (slope < criticalValue_) {
      mapOut.at(type_, *iterator) = 0.2 + 0.8*(slope/criticalValue_);//1.0 - slope / criticalValue_;
    }
    else {
      mapOut.at(type_, *iterator) = 1.0;//0.0;
    }

    if (slope > slopeMax) slopeMax = slope;
    if (slope < slopeMin) slopeMin = slope;
    
    if (start_flag == 0) {mapOut.at(type_, *iterator) = 0.0; start_flag =1;}
    else if(start_flag ==1){mapOut.at(type_, *iterator) = 1.0; start_flag =2;}
    else;


  }
  //ROS_INFO("Slope_MIN/MAX :%.3f, %.3f",slopeMax,slopeMin);
  ROS_DEBUG("slope max = %f", slopeMax);

  return true;
}

bool update_normalvector(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut)//, Eigen::Vector2d center_) 
{
  //! Normal vector positive axis.
  Eigen::Vector3d normalVectorPositiveAxis_ = grid_map::Vector3::UnitZ();

  //! Input layer name.
  std::string inputLayer_ = "elevation";

  //! Output layer name.
  std::string outputLayersPrefix_ = "surface_normal_";
 
  //ROS_INFO_THROTTLE(0.3, "update Normal_vector!");
  std::vector<std::string> normalVectorsLayers;
  normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

  mapOut = mapIn;
  for (const auto& layer : normalVectorsLayers) mapOut.add(layer);
 
    computeWithArea(mapOut, inputLayer_, outputLayersPrefix_);

  return true;
}

void update_traverse(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
{       
    if(update_normalvector(submap,submap))
    {
	if(update_slope(submap,submap)){if(update_roughness1(submap,submap)) {if(update_total(submap,submap,msg))ROS_RED_STREAM("Complete update_traverse in Enable_Traverse");}}
        else ;
    }
    else ; 
}

grid_map::GridMap getSubmapMessage(grid_map::GridMap& original_gridmap, //const grid_map::GridMap& original_gridmap, 
                                    const grid_map::Position& position,
                                    const grid_map::Length& length,
                                    const std::vector<std::string>& layers,
                                    const bool useRaw, bool& isSuccess)
{
    grid_map::Index index;
  
    grid_map::GridMap subMap;
    if (useRaw) 
    {   
        subMap = original_gridmap.getSubmap(position, length, index, isSuccess);
    } else ROS_INFO("useRaw is false.");
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(subMap, layers, message);

    return subMap; 
}

void foothold_detect(const aidin_msgs::pt_array& msg,
                                    const aidin_msgs::three& G_P_F, 
                                    aidin_msgs::four& msg_)
{
    //ROS_INFO("come in foothold");
    int nPoint_ = (msg.nPoint); // array 갯수
    unsigned char continue_check = 0,num = 0;
    float x = G_P_F.a;
    float y = G_P_F.b;
    float z = G_P_F.c;
    //msg_.d = G_P_F.d;
    //float dist ;
    std::vector<double> v_length_dist, v_origin_dist, v_foot_dist; 
    std::vector<double> v_length_score, v_origin_score, v_foot_score, v_total_score; 
    Eigen::Vector3d G_P_ss_;
    G_P_ss_(0) = G_P_F.a;
    G_P_ss_(1) = G_P_F.b;
    G_P_ss_(2) = G_P_F.c;
    // if(nPoint_ < 5) 
    // {
    //     printf("- Submap Not Complete :  nPoint_ : %d !!!!!!\n", nPoint_);
    //     msg_.a = x; msg_.b = y; msg_.c = 0; msg_.d = FailMapData;

    // }
    // else
    // {
        for(int i = 0; i < nPoint_; i++)
        {
            float dist1, dist2, dist3;
            //if((-0.05<msg.array[i].z)&&(msg.array[i].z<0.05))
            {
            dist1 = sqrt(pow(msg.array[i].x - G_P_ss_(0), 2) + pow(msg.array[i].y - G_P_ss_(1), 2) + pow(msg.array[i].z - G_P_ss_(2), 2));;
            v_origin_dist.push_back(dist1);
            dist2 = sqrt(pow(msg.array[i].x - G_P_ss_(0), 2) + pow(msg.array[i].y - G_P_ss_(1), 2));
            v_length_dist.push_back(dist2);
            dist3 = sqrt((msg.array[i].x-x)*(msg.array[i].x-x)
                    +(msg.array[i].y-y)*(msg.array[i].y-y));
            v_foot_dist.push_back(dist3);
            }
        }
        double origin_max = *max_element(v_origin_dist.begin(),v_origin_dist.end());
        double origin_min = *min_element(v_origin_dist.begin(),v_origin_dist.end());
        printf("-origin_max     : %.3f           min   : %.3f       \n",origin_max , origin_min);
        double length_max = *max_element(v_length_dist.begin(),v_length_dist.end());
        double length_min = *min_element(v_length_dist.begin(),v_length_dist.end());

        double foot_max = *max_element(v_foot_dist.begin(),v_foot_dist.end());
        double foot_min = *min_element(v_foot_dist.begin(),v_foot_dist.end());
        
        for(int i = 0; i < nPoint_; i++)
        {  
          //v_origin_score.push_back( ( origin_max ) - ( v_origin_dist[i] ) );
          //v_length_score.push_back( ( v_length_dist[i] ) - (length_max ) ); //max length
          //v_foot_score.push_back( (foot_max ) - ( v_foot_dist[i] / (foot_max-foot_min)) );
          //v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + v_foot_score[i] );
        
          // 나누기는 여기밖에......
          v_origin_score.push_back( ( origin_max / (origin_max-origin_min)) - ( v_origin_dist[i] / (origin_max-origin_min)) );
          v_length_score.push_back( ( v_length_dist[i] / (length_max-length_min)) - (length_max / (length_max-length_min)) ); //max length
          v_foot_score.push_back( (foot_max / (foot_max-foot_min)) - ( v_foot_dist[i] / (foot_max-foot_min)) );
          v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + 2*v_foot_score[i] );
//          std::cout <<"origin"<<v_origin_score<<"length"<<v_length_score;
//        printf("-origin : %.3f,%.3f,%.3f,%.3f    length : %.3f,%.3f,%.3f,%.3f   foot : %.3f   total : %.3f \n",v_origin_score, v_length_score, v_foot_score, v_total_score);
 //         printf("-origin : %.3f,%.3f,%.3f,%.3f", v_total_score);
        }


        while(!continue_check)
        {
            idx_max = std::distance(v_total_score.begin(),max_element(v_total_score.begin(),v_total_score.end()));
        //// exp1 different
//            printf(" spot: (%.3f)\n",consider_length1.b);

            // printf(" msg.array[idx_max].roughness: (%.3f)\n",consider_length1.b-msg.array[idx_max].y);
//            if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.05+0.17<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.05+0.17)&&(0.4<abs(consider_length1.b-msg.array[idx_max].y))&&(abs(consider_length1.b-msg.array[idx_max].y)<1.0)) 
            if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.5<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.5)) 
            {//(0.18<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.22)&&
              
                msg_.a = msg.array[idx_max].x;msg_.b = msg.array[idx_max].y;
                msg_.c = msg.array[idx_max].z;//msg_.d = Target_Foot; 
                printf("msga: %.3f  b: %.3f  c: %.3f \n",msg_.a, msg_.b, msg_.c);
                
                continue_check = 1;
                if(msg.array[idx_max+31].z - msg_.c > 0.05 && msg.array[idx_max+32].z - msg_.c > 0.05 && msg.array[idx_max+33].z - msg_.c > 0.05){
                    idx_max = idx_max-64;
                    printf("i'm running     : 4444           min   : 222    \n");}
                else if (msg.array[idx_max+47].z - msg_.c > 0.05&& msg.array[idx_max+48].z - msg_.c > 0.05 && msg.array[idx_max+49].z - msg_.c > 0.05){
                    idx_max = idx_max-48;
                }
                else if (msg.array[idx_max+63].z - msg_.c > 0.05&& msg.array[idx_max+64].z - msg_.c > 0.05 && msg.array[idx_max+65].z - msg_.c > 0.05){
                    idx_max = idx_max-32;
                }
                else{printf("i'm not:  222(%.3f)    \n"),msg.array[idx_max+30].z;}
                
                // while (fabs(msg.array[idx_max].z- msg_.c)<0.05)
                // {
                //     idx_max = idx_max +16;
                // }
                
                if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
                idx_max = idx_max+16;
                }
                else
                if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
                idx_max = idx_max+16;
                }
                else
                if (fabs(msg.array[idx_max-16].z- msg_.c) >0.05){
                idx_max = idx_max+16;
                }
                else
                msg_.a = msg.array[idx_max].x; msg_.b = msg.array[idx_max].y;
                msg_.c = msg.array[idx_max].z;
            }
            else
            {
                v_total_score[idx_max] = 0;
                num++;
                if(num == 100) {ROS_INFO("May be no map!"); continue_check = 1;}
                idx_max = 256;
            }
            printf("idx_max: %d \n",idx_max);
        }
    continue_check = 0;
}

void submap_select(grid_map::GridMap& original_gridmap, 
                     grid_map::GridMap& submap,
                     const aidin_msgs::three& where_is_searching_area,
                     const aidin_msgs::two& length)
{
    bool useRaw = true ;
    bool isSuccess ;

    grid_map::Position position(where_is_searching_area.a,where_is_searching_area.b);
    grid_map::Length length_(length.a,length.b);
    std::vector<std::string> layers ;//= {"elevation","variance"};
    layers.push_back("elevation");
    layers.push_back("variance");
    submap = getSubmapMessage(original_gridmap, position, length_,layers,useRaw,isSuccess);
}

void Gridmap2pt(grid_map::GridMap& mapOut, aidin_msgs::pt_array& msg)
{
    aidin_msgs::pt_roughness msg_;
    static uint32_t nPoint1 = 0 ;
    nPoint1 = 0 ;
    for (grid_map::GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {
    // Check if this is an empty cell (hole in the map).

    Eigen::Vector3d xyz;
    mapOut.getPosition3("elevation", *iterator, xyz);
    double x,y,z;
    x = (double)(xyz(0,0)); y = (double)(xyz(1,0)); z = (double)(xyz(2,0));
    
    if(!(-0.01<x)&&(x<0.01))
    {
        msg_.x = x; msg_.y = y; msg_.z = z;
    
        msg.array.push_back(msg_);

        nPoint1 = nPoint1 +1;
    }
    else;

    }
    msg.nPoint = nPoint1;
}

void gridtoimage_pre(const grid_map_msgs::GridMap& msg, int num) //son for comparison with current method
{
  ros::NodeHandle nodeHandle_;
  std::string gridMapTopic_;
  std::string filePath_;

  srand((unsigned int)time(NULL));

  filePath_="/home/son/Desktop/dataset/dataset1/elevation_map_previous/num.png";
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

void gridtoimage_cur(const grid_map_msgs::GridMap& msg, int num) //son
{
  ros::NodeHandle nodeHandle_;
  std::string gridMapTopic_;
  std::string filePath_;

  srand((unsigned int)time(NULL));

  filePath_="/home/son/Desktop/dataset/dataset1/elevation_map_current/num.png";
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

// void getdataset(int _id){
//   // grid_map::GridMap raw_map;
//   // grid_map::GridMapRosConverter::fromMessage(Elevation_Map_Copy, raw_map);
//   gridtoimage_pre(Elevation_Map_Copy, _id); // previous method
//   gridtoimage_cur(elevation_map_local, _id); // current method
// }

// void SaveDataset() {
//     // create the output file stream
//     // ofstream file("/home/son/Desktop/dataset/dataset1/dataset.csv");

//     ofstream file("/home/son/Desktop/dataset/dataset1/dataset.csv", ios::app);

//     // check if the file was opened successfully
//     if (!file.is_open()) {
//         cout << "Error opening file!" << endl;
//     }

//     // write the header row
//     // file << "id,global_initial_x,global_initial_y,local_target_x,local_target_y,success_or_failure" << endl;

//     // write some sample data
//     file << id << "," << global_initial_x << "," << global_initial_y << "," << local_target_x << "," << local_target_y << "," << yaw_init << "," << yaw_target << "," << duration << "," << s_or_f << endl;


//     // close the file
//     file.close();

//     cout << "Data saved to file!" << endl;
// }

// void subscriberCallback1 (const  grid_map_msgs::GridMap& msg1)
// {   Elevation_Map_Copy = msg1;
//     mappingstart = 1;}

// void msgCallbackDataset(const dataset_collector::dataset& msg) {
//     id = msg.id;
//     Elevation_Map_Copy = msg.elevation_map_raw;
//     world_x = msg.world_x;
//     world_y = msg.world_y;
//     world_z = msg.world_z; // add 
//     local_target_x = msg.local_target_x;
//     local_target_y = msg.local_target_y;
//     yaw_init = msg.yaw_init;
//     yaw_target = msg.yaw_target;
//     duration = msg.duration;
//     s_or_f = msg.s_or_f;
    

//     std::cout << "======================================" << std::endl;
//     std::cout << "id: " << id << std::endl;
//     std::cout << "global_inital_x: " << global_initial_x << std::endl;
//     std::cout << "global_inital_y: " << global_initial_y << std::endl;
//     std::cout << "local_target_x: " << local_target_x << std::endl;
//     std::cout << "local_target_y: " << local_target_y << std::endl;;
//     std::cout << "yaw_init: " << yaw_init << std::endl;
//     std::cout << "yaw_target: " << yaw_target << std::endl;
//     std::cout << "duration(s): " << duration << std::endl;    

//     if (s_or_f == 1) {
//       std::cout << "result: success" << std::endl;  
//     }
//     else {
//       std::cout << "result: failure" << std::endl;  
//     }

//     SaveDataset();

// //     // mappingstart = 1;

//     grid_map::GridMap elevation_map_local_pre;
//     grid_map::GridMapRosConverter::fromMessage(Elevation_Map_Copy, elevation_map_local_pre);

//     grid_map::GridMap elevation_map_local;
//     elevation_map_local = GetLocalMap(world_x, world_y, world_z);

//     gridtoimage_pre(elevation_map_local_pre, id); // previous method
//     gridtoimage_cur(elevation_map_local, id); // current method
// }

// void msgCallbackZ(const std_msgs::Float64MultiArray& msg) {
//     Z_WorldtoLocal = msg->data[2];
// }

void msgCallbackDataset(const dataset_collector::dataset& msg) {
    id = msg.id;
    world_x_init = msg.world_x_init;
    world_y_init = msg.world_y_init;
    world_z_init = msg.world_z_init; // add 
    Elevation_Map_Copy = msg.elevation_map_raw;
    
    std::cout << "======================================" << std::endl;
    std::cout << "id: " << id << std::endl;
    std::cout << "world_x: " << world_x_init << std::endl;
    std::cout << "world_y: " << world_y_init << std::endl;
    std::cout << "world_z: " << world_z_init << std::endl;

    grid_map::GridMap elevation_map_local;
    elevation_map_local = GetLocalMap(world_x_init, world_y_init, world_z_init); // grid_map::GridMap
    grid_map_msgs::GridMap elevation_map_local_msgs;
    grid_map::GridMapRosConverter::toMessage(elevation_map_local, elevation_map_local_msgs);

    gridtoimage_pre(Elevation_Map_Copy, id); // previous method
    gridtoimage_cur(elevation_map_local_msgs, id); // current method
}

void GetGlobalMap() {
  // Open the bag file
  rosbag::Bag bag;
  bag.open("/home/son/Desktop/dataset/dataset1/global_map_1.bag", rosbag::bagmode::Read);
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
      // grid_map::GridMap elevation_map_global;
      grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, elevation_map_global);
      // gridtoimage_cur(*grid_map_msg, 1);
    }
  }

  // Close the bag file
  bag.close();  
}

grid_map::GridMap GetLocalMap(double _x, double _y, double _z) {

  // Define submap center and size
  grid_map::Position submap_center(_x, _y);  // center of submap in map coordinates
  double submap_size_x = 2.5;  // size of submap in meters
  double submap_size_y = 2.5;  // size of submap in meters
  // grid_map::Length submap_size(submap_size_x, submap_size_y);
  grid_map::Length submap_size(2.5, 2.5);
  bool isSuccess;
  // Extract submap
  grid_map::GridMap _elevation_map_local;
  _elevation_map_local = elevation_map_global.getSubmap(submap_center, submap_size, isSuccess);

  // Add 0.8 to submap data
  ROS_ERROR("%f", _z);
  // _elevation_map_local.add("elevation_inpainted", -_z);

  // ofstream file("/home/son/Desktop/dataset/dataset1/dataset.txt", ios::app);

  // for (grid_map::GridMapIterator iterator(_elevation_map_local); !iterator.isPastEnd(); ++iterator) {
  //       const auto index = *iterator;
  //       // std::cout << ", "<< _elevation_map_local.at("elevation_inpainted", index);
  //       file <<  _elevation_map_local.at("elevation_inpainted", index) << " ";
  //   }
  //       file <<  "minus z";
  for (grid_map::GridMapIterator it(_elevation_map_local); !it.isPastEnd(); ++it) {
    if (_elevation_map_local.isValid(*it, "elevation_inpainted")) {
        _elevation_map_local.at("elevation_inpainted", *it) -= _z;
    }
  }

  // for (grid_map::GridMapIterator iterator(_elevation_map_local); !iterator.isPastEnd(); ++iterator) {
  //       const auto index = *iterator;
  //       std::cout << ", "<< _elevation_map_local.at("elevation_inpainted", index);
  //       file <<  _elevation_map_local.at("elevation_inpainted", index) << " ";
  //   }
  
  // file.close();


  return _elevation_map_local;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dataset_collector");
  ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("/aidin81/elevation_mapping/elevation_map_raw", 100, subscriberCallback1);
  ros::Subscriber sub_dataset = nh.subscribe("/aidin81/dataset", 100, msgCallbackDataset);
  // ros::Subscriber sub_z = nh.subscribe("/aidin81/BodyPos_sim", 100, msgCallbackZ);
  
  GetGlobalMap(); // get grid_map::GridMap elevation_map_global

  // grid_map::GridMap elevation_map_local;
  // elevation_map_local = GetLocalMap(0,0,0); // grid_map::GridMap
  // grid_map_msgs::GridMap elevation_map_local_msgs;
  // grid_map::GridMapRosConverter::toMessage(elevation_map_local, elevation_map_local_msgs);
  // gridtoimage_cur(elevation_map_local_msgs, 1);



  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // if(mappingstart == 1) {
    //   getdataset();
    // }
    // else;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
