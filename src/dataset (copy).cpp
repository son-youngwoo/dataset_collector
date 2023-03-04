#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <ctime>


// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//
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

#define estimationRadius_normal 0.08//0.08
#define estimationRadius_roughness  0.08//0.06//0.08//0.08//0.06
#define criticalValue_roughness  0.05//0.05//0.01//0.007//0.02//0.02
#define roughness_threshold 0.2

#define ROS_RED_STREAM(SSS) ROS_INFO_STREAM("\033[31;1m" << SSS << "\033[0m")

ros::Publisher submap_pub_;
ros::Publisher pub_roughness;
ros::Publisher pub_height;
ros::Publisher normal;
ros::Publisher red;
ros::Publisher pub_foothold;


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
int idx_max;
int sunseo= 1;

// float cor[numrow][numcol]=
// {
//     // {-1,1.5},
//     // {-0.5,1.5},
//     // {0.5,1.5},
//     // {1,1.5},
//     // {-1,1},
//     // {-0.5,1},
//     // {0.5,1},
//     // {1,1},
//     // {-1,0},
//     // {-0.5,0},
//     // {0.5,0},
//     // {1,0},
//     // {-1,-0.5},
//     // {-0.5,-0.5},
//     // {0.5,-0.5},
//     // {1,-0.5}
//     {-0.1,0.5},
//     {-0.5,0.5},
//     {0.5,0.5},
//     {0.1,0.5},
//     {-0.1,0.1},
//     {-0.5,0.1},
//     {0.5,0.1},
//     {0.1,0.1},
//     {-0.1,0},
//     {-0.5,0},
//     {0.5,0},
//     {0.1,0},
//     {-0.1,-0.5},
//     {-0.5,-0.5},
//     {0.5,-0.5},
//     {0.1,-0.5}




// };

void TargetFoot(const aidin_msgs::four& msg)
{
		target_pos.a = msg.a;
		target_pos.b = msg.b;
		target_pos.c = msg.c;

		printf("posy,posz : (%.3f, %.3f)\n",target_pos.b,target_pos.c );

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

void update_heightedge(grid_map::GridMap& submap, aidin_msgs::pt_array& msg)
{       
    ROS_INFO("normal Vector Start!");
        if(update_edge(submap,submap,msg)) ROS_RED_STREAM("Complete update_heightedge in Enable_Traverse");
        else ROS_WARN_STREAM("Not Complete update_traverse in Enable_Traverse");
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
  //  printf("arraynumber     : %d          min   :    \n",nPoint_);
    unsigned char continue_check = 0,num = 0;
    float x = G_P_F.a;
    float y = G_P_F.b;
    float z = G_P_F.c;
//    msg_.d = G_P_F.d;
    //float dist ;
    std::vector<double> v_length_dist, v_origin_dist, v_foot_dist; 
    std::vector<double> v_length_score, v_origin_score, v_foot_score, v_total_score; 
    Eigen::Vector3d G_P_ss_;
    G_P_ss_(0) = G_P_F.a;
    G_P_ss_(1) = G_P_F.b;
    G_P_ss_(2) = G_P_F.c;
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
          //  printf("-origin_max     : %.3f \n",msg.array[i].z);
            }
        }
        double origin_max = *max_element(v_origin_dist.begin(),v_origin_dist.end());
        double origin_min = *min_element(v_origin_dist.begin(),v_origin_dist.end());
//        printf("-origin_max     : %.3f           min   : %.3f       \n",origin_max , origin_min);

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

          v_origin_score.push_back( ( origin_max / (origin_max-origin_min)) - ( v_origin_dist[i] / (origin_max-origin_min)) );
          v_length_score.push_back( ( v_length_dist[i] / (length_max-length_min)) - (length_max / (length_max-length_min)) ); //max length
          v_foot_score.push_back( (foot_max / (foot_max-foot_min)) - ( v_foot_dist[i] / (foot_max-foot_min)) );
          v_total_score.push_back( 1.5*v_origin_score[i] + 2*v_length_score[i] + 2*v_foot_score[i] );

        }


        while(!continue_check)
        {
            idx_max = std::distance(v_total_score.begin(),max_element(v_total_score.begin(),v_total_score.end()));
//            printf("roughness: %.3f \n",msg.array[idx_max].roughness);
        //// exp1 different
            printf(" idx_max: (%d)\n",idx_max);
            if((msg.array[idx_max].roughness== roughness_threshold)&&(-0.05+0.17<msg.array[idx_max].z)&&(msg.array[idx_max].z<0.05+0.17))
            // if((msg.array[idx_max].roughness== roughness_threshold))
            {    
                msg_.a = msg.array[idx_max].x;msg_.b = msg.array[idx_max].y;
                msg_.c = msg.array[idx_max].z;//msg_.d = Target_Foot; 
                printf("msga: %.3f  b: %.3f  c: %.3f \n",msg_.a, msg_.b, msg_.c);
                continue_check = 1;
            }
            else
            {
                v_total_score[idx_max] = 0;
                idx_max = 500;
                num++;
                if(num == 100) {ROS_INFO("May be no map!"); continue_check = 1;}
            }
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

void gridtoimage(const grid_map_msgs::GridMap& msg, int num, int sun)
{
  ros::NodeHandle nodeHandle_;
  std::string gridMapTopic_;
  std::string filePath_;

  srand((unsigned int)time(NULL));
  int rannumb = rand();
//  std::string num_;
  //num_<<num;
  filePath_="/home/kkw/Desktop/imsi_2/num/su_ran.png";
  filePath_.replace(filePath_.find("num"), 3, std::to_string(num));
  filePath_.replace(filePath_.find("su"), 2, std::to_string(sun));
  filePath_.replace(filePath_.find("ran"), 3, std::to_string(rannumb));
  nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/submap3"));
//  nodeHandle_.param("file", filePath_, std::string("/home/kch/Desktop/dataset/nu /grid_map_image.png"));
  ROS_INFO("Saving map received from: %s to file %s.", nodeHandle_.resolveName(gridMapTopic_).c_str(), filePath_.c_str());
  grid_map::GridMap map;
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
  grid_map::GridMapRosConverter::toCvImage(map,"elevation", sensor_msgs::image_encodings::BGR16,-0.5,1, image);
  bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
  ROS_INFO("Success writing image: %s", success?"true":"false");
//  ros::shutdown();
}
 
void subscriberCallback1 (
        const  grid_map_msgs::GridMap& msg1)
{
    grid_map_msgs::GridMap Elevation_Map_Copy;
    Elevation_Map_Copy = msg1;
    grid_map::GridMap raw_map;
    grid_map::GridMapRosConverter::fromMessage(Elevation_Map_Copy, raw_map);
    aidin_msgs::two length;
    grid_map::GridMap submap;
    length.a = 0.525;
    length.b = 0.525;
    float cor[100][2];
    int nu = 0;
    for(nu;nu<9;nu++){
    cor[nu][0]=-1.8+(nu+1)*0.36;
    cor[nu][1]=1.8;}
    for(nu;nu<19;nu++){
    cor[nu][0]=-1.8+(nu-9)*0.36;
    cor[nu][1]=1.8-0.36;}
    for(nu;nu<29;nu++){
    cor[nu][0]=-1.8+(nu-19)*0.36;
    cor[nu][1]=1.8-0.36*2;}
    for(nu;nu<39;nu++){
    cor[nu][0]=-1.8+(nu-29)*0.36;
    cor[nu][1]=1.8-0.36*3;}
    for(nu;nu<49;nu++){
    cor[nu][0]=-1.8+(nu-39)*0.36;
    cor[nu][1]=1.8-0.36*4;}
    for(nu;nu<59;nu++){
    cor[nu][0]=-1.8+(nu-49)*0.36;
    cor[nu][1]=1.8-0.36*5;}
    for(nu;nu<69;nu++){
    cor[nu][0]=-1.8+(nu-59)*0.36;
    cor[nu][1]=1.8-0.36*6;}
    for(nu;nu<79;nu++){
    cor[nu][0]=-1.8+(nu-69)*0.36;
    cor[nu][1]=1.8-0.36*7;}
    for(nu;nu<89;nu++){
    cor[nu][0]=-1.8+(nu-79)*0.36;
    cor[nu][1]=1.8-0.36*8;}
    for(nu;nu<99;nu++){
    cor[nu][0]=-1.8+(nu-89)*0.36;
    cor[nu][1]=1.8-0.36*9;}

    for(int col=0; col<99; ++col){
      
    spot.a=cor[col][0];
    spot.b=cor[col][1];
    spot.c=0.0001;
    submap_select(raw_map, submap, spot, length);
    aidin_msgs::pt_array roughness;
    update_heightedge(submap, roughness);


    grid_map_msgs::GridMap message;
    std::vector<std::string> layers;
    layers.push_back("elevation");
    layers.push_back("variance");
    layers.push_back("traversability_roughness");
    grid_map::GridMapRosConverter::toMessage(submap, layers, message);
    submap_pub_.publish(message);

    foothold_detect(roughness, spot, footholdpredict);

    aidin_msgs::pt_array height;
    Gridmap2pt(submap, height);

    height_ = height;

    normal.publish(spot);
    red.publish(footholdpredict);
    printf("roughness     ");
    TargetFoot(footholdpredict);
//    TargetFoot(spot);
    Footprint(&marker, target_pos);
    pub_foothold.publish(marker);
    if(sunseo<101){
    gridtoimage(message,idx_max,sunseo);}
    else;
    sunseo = ++sunseo;    
    int number;
    std::cin >> number;
    std::cout << "number: "<<number <<'\n';
    }
}




int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "giveme");
  ros::NodeHandle nh;
  ros::NodeHandle pnh_;
  ros::Subscriber sub = pnh_.subscribe("/elevation_mapping/elevation_map_raw",
                            100, subscriberCallback1);
  submap_pub_ = pnh_.advertise<grid_map_msgs::GridMap>("/submap3", 100);
  pub_roughness = pnh_.advertise<aidin_msgs::pt_array>("/roughness_3",1);//190311
  pub_height = pnh_.advertise<aidin_msgs::pt_array>("/submap3_height",1);//190311
  // Create a ROS subscriber for the input point cloud
  normal = pnh_.advertise<aidin_msgs::four>("/Nominal", 100); 
  red = pnh_.advertise<aidin_msgs::four>("/TargetFoot", 100);
  pub_foothold = nh.advertise<visualization_msgs::MarkerArray>("Marker",1);
  ros::spin();
//  ros::shutdown();
}