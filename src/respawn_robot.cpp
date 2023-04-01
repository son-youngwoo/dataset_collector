// start 모드부터 시작하는것
// #include "respawn_robot.h"

// // #define ready 0

// // int cnt[6] = {0, };

// // cnt[ready] 

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 

//     while(ros::ok()) {  

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
//             overturn = 1;
//         }
    
//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // respawn
//                 cnt_respawn++;
                
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("imergency respawn");
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 40) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;  
//                 }
//             }
//             else if (overturn == 0) {

//                 cnt_ready++;

//                 if(cnt_ready == 10) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     StandMode();
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > initialize imu roll, pitch & encoder" << std::endl;
//                     yaw_init = yaw;
//                     PubZeroTorqueFlag(1);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > initialize imu roll, pitch & encoder" << std::endl;
//                     yaw_init = yaw;
//                     InitializeSensor();
//                 }
//                 else if(cnt_ready == 70) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     InitializeYaw();  
//                 }
//                 else if(cnt_ready == 80) { //
//                     PubZeroTorqueFlag(0); 
//                     std::cout << "  > crawl"   << std::endl;  
//                     CrawlMode();
//                 }
//                 // else if(cnt_ready == 100) { // 
//                 //     std::cout << "  > stand"  << std::endl;
//                 //     StandMode();
//                 // }
//                 else if(cnt_ready == 100 ) { //
                    
//                     std::cout << "  > mpc" << std::endl;
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 150) { // 
//                     if(z < 0.4) {
//                         overturn = 1;
//                     } 
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 155) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.15;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
          
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 10) {
//                         PublishVelocity();
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > result: success" << std::endl;
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                     }

//                     std::cout << "  > publish dataset" << std::endl;
//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 1) {
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                         CrawlMode();
//                     }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         std::cout << "=======================================" << std::endl;  
//                     }   
//                 }  
//             }
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }


// base
// #include "respawn_robot.h"

// // #define ready 0

// // int cnt[6] = {0, };

// // cnt[ready] 

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 

//     while(ros::ok()) {  

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
//             overturn = 1;
//         }
    
//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // respawn
//                 cnt_respawn++;
                
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("imergency respawn");
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 40) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;  
//                 }
//             }
//             else if (overturn == 0) {

//                 cnt_ready++;

//                 if(cnt_ready == 10) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     PubZeroTorqueFlag(1);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > initialize imu roll, pitch & encoder" << std::endl;
//                     yaw_init = yaw;
//                     InitializeSensor();
//                 }
//                 else if(cnt_ready == 70) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     InitializeYaw();  
//                 }
//                 else if(cnt_ready == 80) { //
//                     PubZeroTorqueFlag(0); 
//                     std::cout << "  > crawl"   << std::endl;  
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 100) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     StandMode();
//                 }
//                 else if(cnt_ready == 120 ) { //
//                     if(z < 0.4) {
//                         overturn = 1;
//                     } 
//                     std::cout << "  > mpc" << std::endl;
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 135) { // 
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 140) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.15;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
          
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 10) {
//                         PublishVelocity();
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > result: success" << std::endl;
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                     }

//                     std::cout << "  > publish dataset" << std::endl;
//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 1) {
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                         CrawlMode();
//                     }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         std::cout << "=======================================" << std::endl;  
//                     }   
//                 }  
//             }
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }


// best yaw initial 제거
// #include "respawn_robot.h"

// // #define ready 0

// // int cnt[6] = {0, };

// // cnt[ready] 

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 

//     while(ros::ok()) {  

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             if(cnt1 == 20) {
//                 InitializeSensor();
//                 step = 0;
//             }
//         }
    
//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // respawn
//                 cnt_respawn++;
                
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("imergency respawn");
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 40) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;  
//                 }
//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready == 10) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     // PubZeroTorqueFlag(1);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > initialize imu roll, pitch & encoder" << std::endl;
//                     yaw_init = yaw;
//                     // InitializeSensor();
//                 }
//                 else if(cnt_ready == 70) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     // InitializeYaw();  
//                 }
//                 else if(cnt_ready == 80) { //
//                     // PubZeroTorqueFlag(0); 
//                     std::cout << "  > crawl"   << std::endl;  
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 100) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     StandMode();
//                 }
//                 else if(cnt_ready == 120 ) { //
//                     if(z < 0.4) {
//                         overturn = 1;
//                     } 
//                     std::cout << "  > mpc" << std::endl;
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 135) { // 
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 140) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.15;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
          
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 10) {
//                         PublishVelocity();
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > result: success" << std::endl;
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                     }

//                     std::cout << "  > publish dataset" << std::endl;
//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 1) {
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                         CrawlMode();
//                     }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         std::cout << "=======================================" << std::endl;  
//                     }   
//                 }  
//             }
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

// best
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 

//     while(ros::ok()) {  

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             if(cnt1 == 10) {
//                 InitializeSensor();
//             }
//             else if(cnt1 == 70) {
//                 step = 0;
//             }
//         }
    
//         else if (step == 0) { // preparing ...
//             if(overturn == 1) { // respawn
//                 cnt_respawn++;
                
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("imergency respawn");
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 40) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;  
//                 }
//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     StartMode();
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     yaw_init = yaw;
//                     PubZeroTorqueFlag(1);
//                 }
//                 else if(cnt_ready == 25) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     InitializeYaw();  
//                 }
//                 else if(cnt_ready == 45) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     PubZeroTorqueFlag(0); 
//                 }
//                 else if(cnt_ready == 50) { //
//                     std::cout << "  > crawl"   << std::endl;  
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 70) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     StandMode();
//                 }
//                 else if(cnt_ready == 90 ) { //
//                     if(z < 0.4) {
//                         overturn = 1;
//                     } 
//                     std::cout << "  > mpc" << std::endl;
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 100) { // 
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 105) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.15;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
          
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 10) {
//                         PublishVelocity();
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > result: success" << std::endl;
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                     }

//                     std::cout << "  > publish dataset" << std::endl;
//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 1) {
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                         CrawlMode();
//                     }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         std::cout << "=======================================" << std::endl;  
//                     }   
//                 }  
//             }
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

// bbbbest
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 

//     while(ros::ok()) {

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             if(cnt1 == 20) {
//                 std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                 InitializeSensor();
//             }
//             else if(cnt1 == 70) { // 2
//                 std::cout << "  > initialize imu yaw" << std::endl;
//                 InitializeYaw();  
//             }
//             else if(cnt1 == 90) {
//                 step = 0;
//             }
//         }
    
//         else if (step == 0) { // preparing ...
//             if(overturn == 1) { // respawn
//                 cnt_respawn++;
                
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("imergency respawn");
//                     // CrawlMode();
//                     PubZeroTorqueFlag(1);
//                 }
//                 else if (cnt_respawn == 10) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 11) { // 2s
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if (cnt_respawn == 30) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 50) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;  
//                 }
//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     StartMode();
//                 }
//                 else if(cnt_ready == 5) {
//                     std::cout << "  > publish torque" << std::endl;
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     yaw_init = yaw;
//                     PubZeroTorqueFlag(1);
//                 }
//                 else if(cnt_ready == 25) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     InitializeYaw();  
//                 }
//                 else if(cnt_ready == 45) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     PubZeroTorqueFlag(0); 
//                 }
//                 else if(cnt_ready == 50) { //
//                     std::cout << "  > crawl"   << std::endl;  
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 70) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     StandMode();
//                 }
//                 else if(cnt_ready == 90 ) { //
//                     if(z < 0.4) {
//                         overturn = 1;
//                     } 
//                     std::cout << "  > mpc" << std::endl;
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 100) { // 
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 105) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.25;
//                 ROS_ERROR("%f", d);
//                 ROS_ERROR("%f", R_success);

//                 if ( d < R_success) // arrive
//                 {   
//                     ROS_ERROR("here1");
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         ROS_ERROR("here2");
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
          
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 10) {
//                         PublishVelocity();
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.yaw_init = yaw_init;
//                     dataset.yaw_target = yaw_target; 
//                     dataset.duration = cnt_duration/10;
//                     ROS_ERROR("%f", dataset.duration);
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > result: success" << std::endl;
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                     }

//                     std::cout << "  > publish dataset" << std::endl;
//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 1) {
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > publish zero torque" << std::endl;
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                         PubZeroTorqueFlag(1);
//                         // CrawlMode();
//                     }
//                     else if (cnt_respawn == 10) { // 2s
//                         CrawlMode();
//                     }
//                     else if (cnt_respawn == 11) { // 2s
//                         PubZeroTorqueFlag(0);
//                     }
//                     else if (cnt_respawn == 30) {
//                         std::cout << "  > respawn" << std::endl;
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         std::cout << "=======================================" << std::endl;  
//                     }   
//                 }  
//             }
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

//지금까지 best
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 

//     while(ros::ok()) {

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
//             overturn = 1;
//         }

//         // if (step == 2) {
//         //     cnt1++;
//         //     if(cnt1 == 20) {
//         //         std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//         //         InitializeSensor();
//         //     }
//         //     else if(cnt1 == 70) { // 2
//         //         std::cout << "  > initialize imu yaw" << std::endl;
//         //         InitializeYaw();  
//         //     }
//         //     else if(cnt1 == 90) {
//         //         step = 0;
//         //     }
//         // }
//         if(cnt_resetworld >= 5) {
//             std_srvs::Empty resetSrv;
//             resetClient.call(resetSrv);
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             step = 0;
//             cnt_resetworld = 0;
//         }    

//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("imergency respawn");
//                     PubZeroTorqueFlag(1);
//                     cnt_resetworld++;
//                 }
//                 else if (cnt_respawn == 10) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 11) { // 2s
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if (cnt_respawn == 30) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 50) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     cnt_resetworld++;
//                     std::cout << "=======================================" << std::endl;  
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     StartMode();
//                 }
//                 else if(cnt_ready == 5) {
//                     std::cout << "  > publish torque" << std::endl;
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     PubZeroTorqueFlag(1); // torque off
//                 }
//                 else if(cnt_ready == 25) { // 2    
//                     std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                     InitializeSensor();  
//                 }
//                 else if(cnt_ready == 75) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     yaw_init = yaw;
//                     InitializeYaw();  
//                 }
//                 else if(cnt_ready == 90) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     PubZeroTorqueFlag(0); // torque on
//                 }
//                 else if(cnt_ready == 95) { //
//                     std::cout << "  > crawl"   << std::endl;  
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 110) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     StandMode();
//                 }
//                 else if(cnt_ready == 140 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 150) { //
//                     if(z < 0.4) {
//                         overturn = 1;
//                     } 
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 155) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {

//                 // gazebo::event::Events::worldReset(); 

//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         PublishVelocity();
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.20;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.yaw_init = yaw_init;
//                     dataset.yaw_target = yaw_target; 
//                     dataset.duration = cnt_duration/10;
//                     ROS_ERROR("%f", dataset.duration);
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > result: success" << std::endl;
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                     }

//                     std::cout << "  > publish dataset" << std::endl;
//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 10) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         CrawlMode();
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                     }
//                     else if (cnt_respawn == 30) {
//                         std::cout << "  > respawn" << std::endl;
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         std::cout << "=======================================" << std::endl;  
//                     }   
//                 }  
//             }
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

// best
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

//         if(cnt_resetsim == 5) {
//             // PubStateFlag(0);
//             std_srvs::Empty resetSrv;
//             resetsimulationClient.call(resetSrv);
//             // resetworldClient.call(resetSrv);
            
//             ROS_ERROR("!!! Reset Simulation !!!");
//             outfile << "!!! Reset Simulation !!!\n";

//             cnt_resetsim = 0;
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             cnt1 = 0;
//             step = 2;
//             std::cout << "=======================================" << std::endl;
//             outfile << "=======================================\n";  
//         }

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || z_world < -5) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             // if(cnt1 == 15) {
//             //     PubStateFlag(1);
//             // }
//             if(cnt1 == 20) {
//                 std::cout << "First Ready" << std::endl;
//                 std::cout << "  > initialize imu yaw && encoder" << std::endl;
//                 outfile << "First Ready\n";
//                 outfile << "  > initialize imu yaw && encoder\n";
//                 InitializeSensor();
//             }
//             else if(cnt1 == 80) { // 2
//                 std::cout << "  > initialize imu yaw" << std::endl;
//                 outfile << "  > initialize imu yaw\n";
//                 InitializeYaw();  
//             }
//             else if(cnt1 == 100) {
//                 step = 0;
//             }
//         }
//         else if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("Imergency Respawn !!!");
//                     outfile << "!!! Imergency Respawn !!!\n";  
//                     PubZeroTorqueFlag(1);

//                     cnt_resetsim = cnt_resetsim + 1;
//                     ROS_ERROR("%d", cnt_resetsim);
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 21) { // 2s
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if (cnt_respawn == 30) { // 2s
//                     // PubStateFlag(0);
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 50) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;
//                     outfile << "=======================================\n";
//                     std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                     outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     outfile << "===================" << data_id << "===================\n";
//                     outfile << "Ready\n";
//                     outfile << "  > start\n";
//                     StartMode();
//                     // PubResetContactFlag(1);
//                 }
//                 else if(cnt_ready == 2) {
//                     // PubResetContactFlag(0);  
//                 }
//                 else if(cnt_ready == 5) {
//                     std::cout << "  > publish torque" << std::endl;
//                     outfile << "  > publish torque\n";

//                     PubZeroTorqueFlag(0);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     outfile << "  > publish zero torque\n";
//                     // PubStateFlag(1);
//                     PubZeroTorqueFlag(1); // torque off
//                 }
//                 // else if(cnt_ready == 25) { // 2    
//                 //     std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                 //     InitializeSensor();  
//                 // }
//                 else if(cnt_ready == 30) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;
//                     std::cout << "  > yaw_init: " << yaw_init << std::endl;
//                     outfile << "  > yaw_init: " << yaw_init << "\n";
                    
//                     InitializeYaw();
//                 }
//                 else if(cnt_ready == 50) { //
//                     std::cout << "  > crawl"   << std::endl;  \
//                     outfile << "  > crawl\n";
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 51) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     outfile << "  > publish torque\n";
//                     PubZeroTorqueFlag(0); // torque on
//                 }
//                 else if(cnt_ready == 60) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     outfile << "  > stand\n";
//                     StandMode();
//                 }
//                 else if(cnt_ready == 79) {
//                     if(z < 0.5) {
//                         overturn = 1;
//                     } 
//                 }
//                 else if(cnt_ready == 80 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 100) { //
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 105) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         PublishVelocity();
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.30;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.yaw_init = yaw_init;
//                     dataset.yaw_target = yaw_target; 
//                     dataset.duration = cnt_duration/10;
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > duration: " << dataset.duration << std::endl;
//                         outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         std::cout << "  > duration: " << dataset.duration << std::endl;
//                         outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 10) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > Respawn Step\n";
//                         outfile << "  > crawl\n";
//                         CrawlMode();
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                     }
//                     else if (cnt_respawn == 30) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         // PubStateFlag(0);
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         cnt_resetsim = 0;
//                         std::cout << "=======================================" << std::endl;  
//                         outfile << "=======================================\n";
//                         std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                         outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                         ROS_ERROR("%d", cnt_resetsim);
//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }


// 1시간 30분 문제 x 보행 제거
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

//         if(cnt_resetsim == 5) {
//             // PubStateFlag(0);
//             std_srvs::Empty resetSrv;
//             resetsimulationClient.call(resetSrv);
//             // resetworldClient.call(resetSrv);
            
//             ROS_ERROR("!!! Reset Simulation !!!");
//             outfile << "!!! Reset Simulation !!!\n";

//             cnt_resetsim = 0;
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             cnt1 = 0;
//             step = 2;
//             std::cout << "=======================================" << std::endl;
//             outfile << "=======================================\n";  
//         }

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || z_world < -5) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             // if(cnt1 == 15) {
//             //     PubStateFlag(1);
//             // }
//             if(cnt1 == 20) {
//                 std::cout << "First Ready" << std::endl;
//                 std::cout << "  > initialize imu yaw && encoder" << std::endl;
//                 outfile << "First Ready\n";
//                 outfile << "  > initialize imu yaw && encoder\n";
//                 InitializeSensor();
//             }
//             else if(cnt1 == 80) { // 2
//                 std::cout << "  > initialize imu yaw" << std::endl;
//                 outfile << "  > initialize imu yaw\n";
//                 InitializeYaw();  
//             }
//             else if(cnt1 == 100) {
//                 step = 0;
//             }
//         }
//         else if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("Imergency Respawn !!!");
//                     outfile << "!!! Imergency Respawn !!!\n";  
//                     // PubZeroTorqueFlag(1);

//                     cnt_resetsim = cnt_resetsim + 1;
//                     ROS_ERROR("%d", cnt_resetsim);
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 21) { // 2s
//                     // PubZeroTorqueFlag(0);
//                 }
//                 else if (cnt_respawn == 40) { // 2s
//                     StartMode();
//                 }
//                 else if (cnt_respawn == 60) { // 2s
//                     // PubStateFlag(0);
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 80) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;
//                     outfile << "=======================================\n";
//                     std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                     outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     outfile << "===================" << data_id << "===================\n";
//                     outfile << "Ready\n";
//                     outfile << "  > start\n";
//                     // StartMode();
//                     // PubResetContactFlag(1);
//                 }
//                 else if(cnt_ready == 2) {
//                     // PubResetContactFlag(0);  
//                 }
//                 else if(cnt_ready == 5) {
//                     std::cout << "  > publish torque" << std::endl;
//                     outfile << "  > publish torque\n";

//                     // PubZeroTorqueFlag(0);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     outfile << "  > publish zero torque\n";
//                     // PubStateFlag(1);
//                     // PubZeroTorqueFlag(1); // torque off
//                 }
//                 // else if(cnt_ready == 25) { // 2    
//                 //     std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                 //     InitializeSensor();  
//                 // }
//                 else if(cnt_ready == 30) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;
//                     std::cout << "  > yaw_init: " << yaw_init << std::endl;
//                     outfile << "  > yaw_init: " << yaw_init << "\n";
                    
//                     InitializeYaw();
//                 }
//                 else if(cnt_ready == 50) { //
//                     std::cout << "  > crawl"   << std::endl;  \
//                     outfile << "  > crawl\n";
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 51) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     outfile << "  > publish torque\n";
//                     // PubZeroTorqueFlag(0); // torque on
//                 }
//                 else if(cnt_ready == 60) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     outfile << "  > stand\n";
//                     StandMode();
//                 }
//                 else if(cnt_ready == 79) {
//                     if(z < 0.5) {
//                         overturn = 1;
//                     } 
//                 }
//                 else if(cnt_ready == 80 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 100) { //
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 105) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 // PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         // PublishVelocity();
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
//                 R_success = 0.30;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 50) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.yaw_init = yaw_init;
//                     dataset.yaw_target = yaw_target; 
//                     dataset.duration = cnt_duration/10;
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > duration: " << dataset.duration << std::endl;
//                         outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         std::cout << "  > duration: " << dataset.duration << std::endl;
//                         outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;

//                     if (cnt_respawn == 10) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > Respawn Step\n";
//                         outfile << "  > crawl\n";
//                         CrawlMode();
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                     }

//                     if (cnt_respawn == 30) { // 2s
//                         StartMode();
//                     }
//                     else if (cnt_respawn == 50) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         // PubStateFlag(0);
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 70) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         cnt_resetsim = 0;
//                         std::cout << "=======================================" << std::endl;  
//                         outfile << "=======================================\n";
//                         std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                         outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                         ROS_ERROR("%d", cnt_resetsim);
//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }


// 가장 최근
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

//         if(cnt_resetsim == 5) {
//             // PubStateFlag(0);
//             std_srvs::Empty resetSrv;
//             resetsimulationClient.call(resetSrv);
//             // resetworldClient.call(resetSrv);
            
//             ROS_ERROR("!!! Reset Simulation !!!");
//             outfile << "!!! Reset Simulation !!!\n";

//             cnt_resetsim = 0;
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             cnt1 = 0;
//             step = 2;
//             std::cout << "=======================================" << std::endl;
//             outfile << "=======================================\n";  
//         }

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || z_world < -5) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             // if(cnt1 == 15) {
//             //     PubStateFlag(1);
//             // }
//             if(cnt1 == 20) {
//                 std::cout << "First Ready" << std::endl;
//                 std::cout << "  > initialize imu yaw && encoder" << std::endl;
//                 outfile << "First Ready\n";
//                 outfile << "  > initialize imu yaw && encoder\n";
//                 InitializeSensor();
//             }
//             else if(cnt1 == 80) { // 2
//                 std::cout << "  > initialize imu yaw" << std::endl;
//                 outfile << "  > initialize imu yaw\n";
//                 InitializeYaw();  
//             }
//             else if(cnt1 == 100) {
//                 step = 0;
//             }
//         }
//         else if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("Imergency Respawn !!!");
//                     outfile << "!!! Imergency Respawn !!!\n";  
//                     // PubZeroTorqueFlag(1);

//                     cnt_resetsim = cnt_resetsim + 1;
//                     ROS_ERROR("%d", cnt_resetsim);
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 21) { // 2s
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if (cnt_respawn == 40) { // 2s
//                     StartMode();
//                 }
//                 else if (cnt_respawn == 60) { // 2s
//                     // PubStateFlag(0);
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 80) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;
//                     outfile << "=======================================\n";
//                     std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                     outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready > 1) {
//                     outfile2 << JointAngle[0] << ", " << JointAngle[1] << ", "  << JointAngle[2] << ", "  << JointAngle[3] << ", "  << JointAngle[4] << ", "  << JointAngle[5] << ", "  << JointAngle[6] << ", "  << JointAngle[7] << ", "  << JointAngle[8] << ", "  << JointAngle[9] << ", "  << JointAngle[10] << ", "  << JointAngle[11] << ", " <<"\n";
//                 }
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     outfile << "===================" << data_id << "===================\n";
//                     outfile << "Ready\n";
//                     outfile << "  > start\n";
//                     outfile2 << "id: " << data_id << "\n";
//                     outfile2 << roll << ", " << pitch << ", " << yaw << "\n";  
		        
//                     StartMode();
//                     // PubResetContactFlag(1);
//                 }
//                 else if(cnt_ready == 2) {
//                     // PubResetContactFlag(0);  
//                 }
//                 else if(cnt_ready == 5) {
//                     std::cout << "  > publish torque" << std::endl;
//                     outfile << "  > publish torque\n";

//                     PubZeroTorqueFlag(0);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     outfile << "  > publish zero torque\n";
//                     // PubStateFlag(1);
//                     // PubZeroTorqueFlag(1); // torque off
//                 }
//                 // else if(cnt_ready == 25) { // 2    
//                 //     std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                 //     InitializeSensor();  
//                 // }
//                 else if(cnt_ready == 30) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;
//                     // GetInitialYaw();
//                     std::cout << "  > yaw_init: " << yaw_init << std::endl;
//                     outfile << "  > yaw_init: " << yaw_init << "\n";
                    
//                     InitializeYaw();
//                 }
//                 else if(cnt_ready == 50) { //
//                     std::cout << "  > crawl"   << std::endl;
//                     outfile << "  > crawl\n";
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 51) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     outfile << "  > publish torque\n";
//                     PubZeroTorqueFlag(0); // torque on
//                 }
//                 else if(cnt_ready == 60) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     outfile << "  > stand\n";
//                     // StandMode();
//                 }
//                 else if(cnt_ready == 79) {
//                     if(z < 0.5) {
//                         // overturn = 1;
//                     }
//                 }
//                 else if(cnt_ready == 80 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 100) { //
//                     std_msgs::Float32 first_z;
//                     first_z.data = z_world;
//                     pub_firstz.publish(first_z);
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 105) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         PublishVelocity(0.3);
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((global_x_tar - global_x)*(global_x_tar - global_x) + (global_y_tar - global_y)*(global_y_tar - global_y));
//                 R_success = 0.30;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     dataset.id = data_id; // dataset id add
//                     dataset.elevation_map_raw = elevation_map_raw;
//                     dataset.global_initial_x = x_init; // world base
//                     dataset.global_initial_y = y_init; // world base
//                     dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     dataset.yaw_init = yaw_init;
//                     dataset.yaw_target = yaw_target; 
//                     dataset.duration = cnt_duration/10;
//                     dataset.s_or_f = s_or_f;

//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         std::cout << "  > duration: " << dataset.duration << std::endl;
//                         outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         std::cout << "  > duration: " << dataset.duration << std::endl;
//                         outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;
//                     if (cnt_respawn == 10) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > Respawn Step\n";
//                         outfile << "  > crawl\n";
//                         PublishVelocity(0);
//                     }
//                     else if (cnt_respawn == 30) { // 2s
//                         CrawlMode();
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                     }

//                     if (cnt_respawn == 50) { // 2s
//                         // StartMode();
//                     }
//                     else if (cnt_respawn == 70) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         // PubStateFlag(0);
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 90) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         cnt_resetsim = 0;
//                         std::cout << "=======================================" << std::endl;  
//                         outfile << "=======================================\n";
//                         std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                         outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                         ROS_ERROR("%d", cnt_resetsim);
//                         outfile2 << "=========================================================================================================================="<< "\n";  
//                         cnt = 0;

//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

// best
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

//         if(cnt_resetsim == 5) {
//             // PubStateFlag(0);
//             std_srvs::Empty resetSrv;
//             resetsimulationClient.call(resetSrv);
//             // resetworldClient.call(resetSrv);
            
//             ROS_ERROR("!!! Reset Simulation !!!");
//             outfile << "!!! Reset Simulation !!!\n";

//             cnt_resetsim = 0;
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             cnt1 = 0;
//             step = 2;
//             std::cout << "=======================================" << std::endl;
//             outfile << "=======================================\n";  
//         }

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || z_world < -5) { 
//             overturn = 1;
//         }

//         if (step == 2) {
//             cnt1++;
//             // if(cnt1 == 15) {
//             //     PubStateFlag(1);
//             // }
//             if(cnt1 == 20) {
//                 std::cout << "First Ready" << std::endl;
//                 std::cout << "  > initialize imu yaw && encoder" << std::endl;
//                 outfile << "First Ready\n";
//                 outfile << "  > initialize imu yaw && encoder\n";
//                 InitializeSensor();
//             }
//             else if(cnt1 == 80) { // 2
//                 std::cout << "  > initialize imu yaw" << std::endl;
//                 outfile << "  > initialize imu yaw\n";
//                 InitializeYaw();  
//             }
//             else if(cnt1 == 100) {
//                 step = 0;
//             }
//         }
//         else if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("Imergency Respawn !!!");
//                     outfile << "!!! Imergency Respawn !!!\n";  
//                     // PubZeroTorqueFlag(1);

//                     cnt_resetsim = cnt_resetsim + 1;
//                     ROS_ERROR("%d", cnt_resetsim);
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 21) { // 2s
//                     PubZeroTorqueFlag(0);
//                 }
//                 else if (cnt_respawn == 40) { // 2s
//                     StartMode();
//                 }
//                 else if (cnt_respawn == 60) { // 2s
//                     // PubStateFlag(0);
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 80) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;
//                     outfile << "=======================================\n";
//                     std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                     outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 if(cnt_ready > 1) {
//                     outfile2 << JointAngle[0] << ", " << JointAngle[1] << ", "  << JointAngle[2] << ", "  << JointAngle[3] << ", "  << JointAngle[4] << ", "  << JointAngle[5] << ", "  << JointAngle[6] << ", "  << JointAngle[7] << ", "  << JointAngle[8] << ", "  << JointAngle[9] << ", "  << JointAngle[10] << ", "  << JointAngle[11] << ", " <<"\n";
//                 }
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     outfile << "===================" << data_id << "===================\n";
//                     outfile << "Ready\n";
//                     outfile << "  > start\n";
//                     outfile2 << "id: " << data_id << "\n";
//                     outfile2 << roll << ", " << pitch << ", " << yaw << "\n";  
		        
//                     StartMode();
//                     // PubResetContactFlag(1);
//                 }
//                 else if(cnt_ready == 2) {
//                     // PubResetContactFlag(0);  
//                 }
//                 else if(cnt_ready == 5) {
//                     std::cout << "  > publish torque" << std::endl;
//                     outfile << "  > publish torque\n";

//                     PubZeroTorqueFlag(0);
//                 }
//                 else if(cnt_ready == 20) { // 
//                     std::cout << "  > publish zero torque" << std::endl;
//                     outfile << "  > publish zero torque\n";
//                     // PubStateFlag(1);
//                     // PubZeroTorqueFlag(1); // torque off
//                 }
//                 // else if(cnt_ready == 25) { // 2    
//                 //     std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                 //     InitializeSensor();  
//                 // }
//                 else if(cnt_ready == 30) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;
//                     // GetInitialYaw();
//                     std::cout << "  > yaw_init: " << yaw_init << std::endl;
//                     outfile << "  > yaw_init: " << yaw_init << "\n";
                    
//                     InitializeYaw();
//                 }
//                 else if(cnt_ready == 50) { //
//                     std::cout << "  > crawl"   << std::endl;
//                     outfile << "  > crawl\n";
//                     CrawlMode();
//                 }
//                 else if(cnt_ready == 51) { //
//                     std::cout << "  > publish torque"   << std::endl;  
//                     outfile << "  > publish torque\n";
//                     PubZeroTorqueFlag(0); // torque on
//                 }
//                 else if(cnt_ready == 60) { // 
//                     std::cout << "  > stand"  << std::endl;
//                     outfile << "  > stand\n";
//                     // StandMode();
//                 }
//                 else if(cnt_ready == 79) {
//                     if(z < 0.5) {
//                         // overturn = 1;
//                     }
//                 }
//                 else if(cnt_ready == 80 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 120) { //
//                     std_msgs::Float32 first_z;
//                     first_z.data = z_world;
//                     pub_firstz.publish(first_z);
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 125) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         PublishVelocity(0.3);
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((global_x_tar - global_x)*(global_x_tar - global_x) + (global_y_tar - global_y)*(global_y_tar - global_y));
//                 R_success = 0.10;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     // dataset.id = data_id; // dataset id add
//                     // dataset.elevation_map_raw = elevation_map_raw;
//                     // dataset.global_initial_x = x_init; // world base
//                     // dataset.global_initial_y = y_init; // world base
//                     // dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     // dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     // dataset.yaw_init = yaw_init;
//                     // dataset.yaw_target = yaw_target; 
//                     // dataset.duration = cnt_duration/10;
//                     // dataset.s_or_f = s_or_f;

//                     dataset.id = data_id; // dataset id add
//                     dataset.world_x_init = x_init;
//                     dataset.world_y_init = y_init;
//                     dataset.world_z_init = z_init;
//                     dataset.global_x_tar = global_x_tar;
//                     dataset.global_y_tar = global_y_tar;
//                     dataset.isSuccess = s_or_f;
//                     dataset.elevation_map_realtime = elevation_map_raw;
                    
//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                     //     std::cout << "  > duration: " << dataset.duration << std::endl;
//                     //     outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         // std::cout << "  > duration: " << dataset.duration << std::endl;
//                         // outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;
//                     if (cnt_respawn == 10) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > Respawn Step\n";
//                         outfile << "  > crawl\n";
//                         PublishVelocity(0);
//                     }
//                     else if (cnt_respawn == 30) { // 2s
//                         CrawlMode();
//                         PublishZeroVelocity();
//                         PublishZeroPath();
//                     }

//                     if (cnt_respawn == 50) { // 2s
//                         // StartMode();
//                     }
//                     else if (cnt_respawn == 70) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         // PubStateFlag(0);
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 90) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         cnt_resetsim = 0;
//                         std::cout << "=======================================" << std::endl;  
//                         outfile << "=======================================\n";
//                         std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                         outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                         ROS_ERROR("%d", cnt_resetsim);
//                         outfile2 << "=========================================================================================================================="<< "\n";  
//                         cnt = 0;

//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

//
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

//         if(cnt_resetsim == 5) {
//             // PubStateFlag(0);
//             std_srvs::Empty resetSrv;
//             resetsimulationClient.call(resetSrv);
//             // resetworldClient.call(resetSrv);
            
//             ROS_ERROR("!!! Reset Simulation !!!");
//             outfile << "!!! Reset Simulation !!!\n";

//             cnt_resetsim = 0;
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             cnt1 = 0;
//             step = 0;
//             std::cout << "=======================================" << std::endl;
//             outfile << "=======================================\n";  
//         }

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || z_world < -5) { 
//             overturn = 1;
//         }

//         // if (step == 2) {
//         //     cnt1++;
//         //     // if(cnt1 == 15) {
//         //     //     PubStateFlag(1);
//         //     // }
//         //     if(cnt1 == 20) {
//         //         std::cout << "First Ready" << std::endl;
//         //         std::cout << "  > initialize imu yaw && encoder" << std::endl;
//         //         outfile << "First Ready\n";
//         //         outfile << "  > initialize imu yaw && encoder\n";
//         //         InitializeSensor();
//         //     }
//         //     else if(cnt1 == 80) { // 2
//         //         std::cout << "  > initialize imu yaw" << std::endl;
//         //         outfile << "  > initialize imu yaw\n";
//         //         InitializeYaw();  
//         //     }
//         //     else if(cnt1 == 100) {
//         //         step = 0;
//         //     }
//         // }
//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("Imergency Respawn !!!");
//                     outfile << "!!! Imergency Respawn !!!\n";  
//                     // PubZeroTorqueFlag(1);

//                     cnt_resetsim = cnt_resetsim + 1;
//                     ROS_ERROR("%d", cnt_resetsim);
//                     CrawlMode();
//                 }
//                 // else if (cnt_respawn == 20) { // 2s
//                     // CrawlMode();
//                 // }
//                 // else if (cnt_respawn == 21) { // 2s
//                     // PubZeroTorqueFlag(0);
//                     // Respawn();
//                 // }
//                 // else if (cnt_respawn == 40) { // 2s
//                 //     StartMode();
//                 // }
//                 else if (cnt_respawn == 20) { // 2s
//                     // PubStateFlag(0);
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 40) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;
//                     outfile << "=======================================\n";
//                     std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                     outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;
//                 // if(cnt_ready > 1) {
//                 //     outfile2 << JointAngle[0] << ", " << JointAngle[1] << ", "  << JointAngle[2] << ", "  << JointAngle[3] << ", "  << JointAngle[4] << ", "  << JointAngle[5] << ", "  << JointAngle[6] << ", "  << JointAngle[7] << ", "  << JointAngle[8] << ", "  << JointAngle[9] << ", "  << JointAngle[10] << ", "  << JointAngle[11] << ", " <<"\n";
//                 // }
//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     outfile << "===================" << data_id << "===================\n";
//                     outfile << "Ready\n";
//                     outfile << "  > start\n";
//                     outfile2 << "id: " << data_id << "\n";
//                     outfile2 << roll << ", " << pitch << ", " << yaw << "\n";  
		        
//                     // StartMode();
//                     // PubResetContactFlag(1);
//                 }
//                 // else if(cnt_ready == 2) {
//                 //     // PubResetContactFlag(0);  
//                 // }
//                 // else if(cnt_ready == 5) {
//                 //     std::cout << "  > publish torque" << std::endl;
//                 //     outfile << "  > publish torque\n";

//                 //     PubZeroTorqueFlag(0);
//                 // }
//                 // else if(cnt_ready == 20) { // 
//                 //     std::cout << "  > publish zero torque" << std::endl;
//                 //     outfile << "  > publish zero torque\n";
//                 //     // PubStateFlag(1);
//                 //     // PubZeroTorqueFlag(1); // torque off
//                 // }
//                 // else if(cnt_ready == 25) { // 2    
//                 //     std::cout << "  > initialize imu roll, pitch && encoder" << std::endl;
//                 //     InitializeSensor();  
//                 // }
//                 else if(cnt_ready == 10) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;
//                     // GetInitialYaw();
//                     std::cout << "  > yaw_init: " << yaw_init << std::endl;
//                     outfile << "  > yaw_init: " << yaw_init << "\n";
                    
//                     InitializeYaw();
//                 }
//                 else if(cnt_ready == 30) { //
//                     std::cout << "  > crawl"   << std::endl;
//                     outfile << "  > crawl\n";
//                     CrawlMode();
//                 }
//                 // else if(cnt_ready == 51) { //
//                 //     std::cout << "  > publish torque"   << std::endl;  
//                 //     outfile << "  > publish torque\n";
//                 //     PubZeroTorqueFlag(0); // torque on
//                 // }
//                 // else if(cnt_ready == 60) { // 
//                 //     std::cout << "  > stand"  << std::endl;
//                 //     outfile << "  > stand\n";
//                 //     // StandMode();
//                 // }
//                 // else if(cnt_ready == 79) {
//                 //     if(z < 0.5) {
//                 //         // overturn = 1;
//                 //     }
//                 // }
//                 else if(cnt_ready == 60 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 90) { //
//                     std_msgs::Float32 first_z;
//                     first_z.data = z_world;
//                     pub_firstz.publish(first_z);
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 95) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         PubStateFlag(0);
//                         PubRotationFlag(1);
//                         PublishVelocity(0.15);
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((global_x_tar - global_x)*(global_x_tar - global_x) + (global_y_tar - global_y)*(global_y_tar - global_y));
//                 R_success = 0.10;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {
//                     // dataset.id = data_id; // dataset id add
//                     // dataset.elevation_map_raw = elevation_map_raw;
//                     // dataset.global_initial_x = x_init; // world base
//                     // dataset.global_initial_y = y_init; // world base
//                     // dataset.local_target_x = rand_x_tar - x_init; // robot base
//                     // dataset.local_target_y = rand_y_tar - y_init; // robot base
//                     // dataset.yaw_init = yaw_init;
//                     // dataset.yaw_target = yaw_target; 
//                     // dataset.duration = cnt_duration/10;
//                     // dataset.s_or_f = s_or_f;

//                     dataset.id = data_id; // dataset id add
//                     dataset.world_x_init = x_init;
//                     dataset.world_y_init = y_init;
//                     dataset.world_z_init = z_init;
//                     dataset.global_x_tar = global_x_tar;
//                     dataset.global_y_tar = global_y_tar;
//                     dataset.isSuccess = s_or_f;
//                     dataset.elevation_map_realtime = elevation_map_raw;
                    
//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         // std::cout << "  > duration: " << dataset.duration << std::endl;
//                         // outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         // std::cout << "  > duration: " << dataset.duration << std::endl;
//                         // outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;
//                     if (cnt_respawn == 1) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > Respawn Step\n";
//                         outfile << "  > crawl\n";
//                         CrawlMode();
//                         PublishZeroPath();
//                         PublishZeroVelocity();
//                     }
//                     // else if (cnt_respawn == 30) { // 2s
//                         // CrawlMode();
//                         // PublishZeroVelocity();
//                         // PublishZeroPath();
//                     // }
// // 
//                     // if (cnt_respawn == 50) { // 2s
//                         // StartMode();
//                     // }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         // PubStateFlag(0);
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 90) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         cnt_resetsim = 0;
//                         std::cout << "=======================================" << std::endl;  
//                         outfile << "=======================================\n";
//                         std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                         outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                         ROS_ERROR("%d", cnt_resetsim);
//                         outfile2 << "=========================================================================================================================="<< "\n";  
//                         cnt = 0;

//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

// best
// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

//         if(cnt_resetsim == 5) {
//             // PubStateFlag(0);
//             std_srvs::Empty resetSrv;
//             resetsimulationClient.call(resetSrv);
//             // resetworldClient.call(resetSrv);
            
//             ROS_ERROR("!!! Reset Simulation !!!");
//             outfile << "!!! Reset Simulation !!!\n";

//             cnt_resetsim = 0;
//             cnt_ready = 0;
//             cnt_s_or_f = 0;
//             cnt_path = 0;
//             cnt_vel = 0;
//             cnt_data = 0;
//             cnt_respawn = 0;
//             cnt_duration = 0;
//             get_s_or_f = 0;
//             overturn = 0;
//             cnt1 = 0;
//             step = 0;
//             std::cout << "=======================================" << std::endl;
//             outfile << "=======================================\n";  
//         }

//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || z_world < -5) { 
//             overturn = 1;
//         }

//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;
//                 if (cnt_respawn == 1) { // 0.1s
//                     ROS_ERROR("Imergency Respawn !!!");
//                     outfile << "!!! Imergency Respawn !!!\n";  

//                     cnt_resetsim = cnt_resetsim + 1;
//                     ROS_ERROR("%d", cnt_resetsim);
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 20) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 40) { // 4s
//                     cnt_ready = 0;
//                     cnt_s_or_f = 0;
//                     cnt_path = 0;
//                     cnt_vel = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                     get_s_or_f = 0;
//                     overturn = 0;
//                     step = 0;
//                     std::cout << "=======================================" << std::endl;
//                     outfile << "=======================================\n";
//                     std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                     outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;

//                 if(cnt_ready == 1) {
//                     std::cout << "===================" << data_id << "===================" << std::endl;
//                     std::cout << "Ready" << std::endl;
//                     std::cout << "  > start" << std::endl;
//                     outfile << "===================" << data_id << "===================\n";
//                     outfile << "Ready\n";
//                     outfile << "  > start\n";
//                     outfile2 << "id: " << data_id << "\n";
//                     outfile2 << roll << ", " << pitch << ", " << yaw << "\n";  
//                 }
//                 else if(cnt_ready == 10) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;
//                     // GetInitialYaw();
//                     std::cout << "  > yaw_init: " << yaw_init << std::endl;
//                     outfile << "  > yaw_init: " << yaw_init << "\n";
                    
//                     InitializeYaw();
//                 }
//                 else if(cnt_ready == 30) { //
//                     std::cout << "  > crawl"   << std::endl;
//                     outfile << "  > crawl\n";
//                     CrawlMode();
//                     PubTargetFlag(1);
//                 }
//                 else if(cnt_ready == 60 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 90) { //
//                     std_msgs::Float32 first_z;
//                     first_z.data = z_world;
//                     pub_firstz.publish(first_z);
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 95) { // 
//                     std::cout << "Go" << std::endl;
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_s_or_f == 0) {
                
//                 print_check(cnt_s_or_f, "  > moving ...");

//                 cnt_duration++;
//                 cnt_path++;
//                 PublishPath(cnt_path);
                
//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_vel++;
//                     if (cnt_vel == 20) {
//                         PubTargetFlag(3);
//                     }
//                 }

//                 cnt_s_or_f++;

//                 d = sqrt((global_x_tar - global_x)*(global_x_tar - global_x) + (global_y_tar - global_y)*(global_y_tar - global_y));
//                 R_success = 0.05;

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || z < 0.3) { // arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else { // arrive -> success  
//                         get_s_or_f = 1;
//                         s_or_f = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;
//                     }
//                     else if(cnt_s_or_f == 150) { // not arrive -> fail
//                         get_s_or_f = 1;
//                         s_or_f = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_s_or_f = 0;        
//                     }
//                 }
//             }

//             else if (get_s_or_f == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {

//                     dataset.id = data_id; // dataset id add
//                     dataset.world_x_init = x_init;
//                     dataset.world_y_init = y_init;
//                     dataset.world_z_init = z_init;
//                     dataset.global_x_tar = global_x_tar;
//                     dataset.global_y_tar = global_y_tar;
//                     dataset.isSuccess = s_or_f;
//                     dataset.elevation_map_realtime = elevation_map_raw;
                    
//                     pub_dataset.publish(dataset);
                    
//                     if (s_or_f == 1) {
//                         // std::cout << "  > duration: " << dataset.duration << std::endl;
//                         // outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         // std::cout << "  > duration: " << dataset.duration << std::endl;
//                         // outfile << "  > duration: " << dataset.duration << "\n";
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;
//                     if (cnt_respawn == 1) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > Respawn Step\n";
//                         outfile << "  > crawl\n";
//                         CrawlMode();
//                         PublishZeroPath();
//                         PublishZeroVelocity();
//                     }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 90) {
//                         data_id++;
//                         cnt_ready = 0;
//                         cnt_s_or_f = 0;
//                         cnt_path = 0;
//                         cnt_vel = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         get_s_or_f = 0;
//                         overturn = 0;
//                         step = 0;
//                         cnt_resetsim = 0;
//                         std::cout << "=======================================" << std::endl;  
//                         outfile << "=======================================\n";
//                         std::cout << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << std::endl;
//                         outfile << "  > respawn x: " << rand_x_init << " / respawn y: " << rand_y_init << "\n";
//                         ROS_ERROR("%d", cnt_resetsim);
//                         outfile2 << "=========================================================================================================================="<< "\n";  
//                         cnt = 0;

//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

// best

// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);

//     ros::Rate rate(10); 
    
//     while(ros::ok()) {

        
//         if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || cur_z_world > height_limit) { 
//             overturn = 1;
//         }

//         if (step == 0) { // preparing ...
//             if(overturn == 1) { // Imergency Respawn
//                 cnt_respawn++;

//                 if (cnt_respawn == 1) { // 0.1s

//                     if(cnt_resetsim == 5) {

//                         ROS_ERROR("!!! Reset Simulation !!!");

//                         outfile << "========================\n";  
//                         outfile << "!!! Reset Simulation !!!\n";
//                         outfile << "========================\n";  
                        
//                         std_srvs::Empty resetSrv;
//                         resetsimulationClient.call(resetSrv);
                        
//                         step = 0;
                        
//                         get_isSuccess = false;
//                         overturn = false;
                        
//                         cnt_resetsim = 0;
//                         cnt_ready = 0;
//                         cnt_path = 0;
//                         cnt_forward = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                     }
//                     else {
//                         cnt_resetsim = cnt_resetsim + 1;

//                         ROS_ERROR("!!! Imergency Respawn %d !!!", cnt_resetsim);

//                         outfile << "=========================\n"; 
//                         outfile << "!!! Imergency Respawn " << cnt_resetsim << " " << "!!!\n";
//                         outfile << "=========================\n"; 
//                     }
//                 }
//                 else if (cnt_respawn == 15) { // 2s
//                     CrawlMode();
//                 }
//                 else if (cnt_respawn == 35) { // 2s
//                     Respawn();
//                 }
//                 else if (cnt_respawn == 55) { // 4s
//                     step = 0;

//                     get_isSuccess = false;
//                     overturn = false;

//                     cnt_ready = 0;
//                     cnt_path = 0;
//                     cnt_forward = 0;
//                     cnt_data = 0;
//                     cnt_respawn = 0;
//                     cnt_duration = 0;
//                 }

//             }
//             else if (overturn == 0) {
//                 cnt_ready++;

//                 if(cnt_ready == 1) {
//                     std::cout << "====================" << data_id << "====================" << std::endl;
//                     outfile << "====================" << data_id << "====================\n";
//                     std::cout << "Preparing Step" << std::endl;
//                     outfile << "Preparing Step\n";
//                 }
//                 else if(cnt_ready == 10) { // 2
//                     std::cout << "  > initialize imu yaw" << std::endl;
//                     outfile << "  > initialize imu yaw\n";
                    
//                     yaw_init = yaw;

//                     InitializeYaw();
//                     PubTargetFlag(1);
//                 }
//                 else if(cnt_ready == 25 ) { // 
//                     std::cout << "  > mpc" << std::endl;
//                     outfile << "  > mpc\n";
//                     MPCMode();
//                 }
//                 else if(cnt_ready == 55) { //
//                     ResetElevationMap(); 
//                 }
//                 else if(cnt_ready == 60) { // 
//                     std::cout << "Tracking Step" << std::endl;
//                     outfile << "  > Tracking Step\n";
//                     elevation_map_raw = elevation_map_raw_copy;
//                     step = 1;
//                 }
//             }

//         }
        
//         else if(step == 1) {
//             if (get_isSuccess == 0) {
                
//                 cnt_duration++;
//                 cnt_path++;

//                 PublishPath(cnt_path);
                
//                 print_check(cnt_duration, "  > moving ...");

//                 if(abs(yaw_target - _yaw_target) < 0.01) {
//                     cnt_forward++;
//                     if (cnt_forward == 20) {
//                         PubTargetFlag(3);
//                     }
//                 }

//                 d = sqrt((global_x_tar - global_x_cur)*(global_x_tar - global_x_cur) + (global_y_tar - global_y_cur)*(global_y_tar - global_y_cur));

//                 if ( d < R_success) // arrive
//                 {   
//                     if (overturn == 1 || cur_z_est < 0.3) { // arrive -> fail
//                         get_isSuccess = 1;
//                         isSuccess = 0;
//                     }
//                     else { // arrive -> success  
//                         get_isSuccess = 1;
//                         isSuccess = 1;
//                     }

//                 }
//                 else { //not arrive
//                     if (overturn == 1) { // not arrive -> fail
//                         get_isSuccess = 1;
//                         isSuccess = 0;
//                     }
//                     else if(cnt_duration == 150) { // not arrive -> fail
//                         get_isSuccess = 1;
//                         isSuccess = 0;  
//                     }
//                     else { // not arrive -> wait
//                         get_isSuccess = 0;        
//                     }
//                 }
//             }

//             else if (get_isSuccess == 1) {
//                 cnt_data++;

//                 if (cnt_data == 1) {

//                     dataset.id = data_id;
//                     dataset.world_x_init = x_init;
//                     dataset.world_y_init = y_init;
//                     dataset.world_z_init = z_init;
//                     dataset.yaw_init = yaw_init;
//                     dataset.global_x_tar = global_x_tar;
//                     dataset.global_y_tar = global_y_tar;
//                     dataset.duration = cnt_duration/10;
//                     dataset.isSuccess = isSuccess;
//                     dataset.elevation_map_realtime = elevation_map_raw;
                    
//                     pub_dataset.publish(dataset);
                    
//                     std::cout << "  > duration: " << dataset.duration << std::endl;
//                     outfile << "  > duration: " << dataset.duration << "\n";
                    
//                     if (isSuccess == 1) {   
//                         std::cout << "  > result: success" << std::endl;
//                         outfile << "  > result: success\n";
//                     }
//                     else {
//                         std::cout << "  > result: failure" << std::endl;
//                         outfile << "  > result: failure\n";
//                     }

//                 }
//                 else if (cnt_data > 1) {
//                     cnt_respawn++;
//                     if (cnt_respawn == 1) { // 2s
//                         std::cout << "Respawn Step" << std::endl;
//                         outfile << "Respawn Step\n";
//                         std::cout << "  > crawl" << std::endl;
//                         outfile << "  > crawl\n";
//                         CrawlMode();
//                         PublishZeroPath();
//                         PublishZeroVelocity();
//                     }
//                     else if (cnt_respawn == 20) {
//                         std::cout << "  > respawn" << std::endl;
//                         outfile << "  > respawn\n";
//                         std::cout << "=========================================" << std::endl;  
//                         outfile << "=========================================\n";
//                         Respawn();
//                     }
//                     else if (cnt_respawn == 40) {
//                         data_id++;
                        
//                         step = 0;
                        
//                         get_isSuccess = false;
//                         overturn = false;
                        
//                         cnt_ready = 0;
//                         cnt_path = 0;
//                         cnt_forward = 0;
//                         cnt_data = 0;
//                         cnt_respawn = 0;
//                         cnt_duration = 0;
//                         cnt_resetsim = 0;

//                     }   
//                 }  
//             }
//         }


//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }




#include "respawn_robot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "respawn_robot");

    ros::NodeHandle nh;

    ROSInit(nh);

    robot_namespace = argv[1]; // get robot_namespace from .launch 

    ofstream outfile("/home/son/Desktop/dataset/dataset1/terminal_output_" + robot_namespace + ".txt");

    ros::Rate rate(10); 
    
    while(ros::ok()) {
        if (step == 0) { // preparing ...
            if(overturn == 1) { // Imergency Respawn
                cnt_respawn++;

                if (cnt_respawn == 1) { // 0.1s

                    if(cnt_resetsim == 30) {

                        ROS_ERROR("!!! Reset Simulation !!!");

                        outfile << "========================\n";  
                        outfile << "!!! Reset Simulation !!!\n";
                        outfile << "========================\n";  
                        
                        std_srvs::Empty resetSrv;
                        resetsimulationClient.call(resetSrv);
                        
                        step = 0;
                        
                        get_isSuccess = false;
                        overturn = false;
                        
                        cnt_resetsim = 0;
                        cnt_ready = 0;
                        cnt_path = 0;
                        cnt_forward = 0;
                        cnt_data = 0;
                        cnt_respawn = 0;
                        cnt_duration = 0;
                    }
                    else {
                        cnt_resetsim = cnt_resetsim + 1;

                        // ROS_ERROR("!!! Imergency Respawn %d !!!", cnt_resetsim);

                        outfile << "=========================\n"; 
                        outfile << "!!! Imergency Respawn " << cnt_resetsim << " " << "!!!\n";
                        outfile << "=========================\n"; 
                    }
                }
                else if (cnt_respawn == 15) { // 2s
                    CrawlMode();
                }
                else if (cnt_respawn == 35) { // 2s
                    Respawn();
                }
                else if (cnt_respawn == 55) { // 4s
                    step = 0;

                    get_isSuccess = false;
                    overturn = false;

                    cnt_ready = 0;
                    cnt_path = 0;
                    cnt_forward = 0;
                    cnt_data = 0;
                    cnt_respawn = 0;
                    cnt_duration = 0;
                }

            }
            else if (overturn == 0) {
                cnt_ready++;

                if(cnt_ready == 1) {
                    // std::cout << "====================" << data_id << "====================" << std::endl;
                    outfile << "====================" << data_id << "====================\n";
                    // std::cout << "Preparing Step" << std::endl;
                    outfile << "Preparing Step\n";
                }
                else if(cnt_ready == 10) { // 2

                    if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || cur_z_world > height_limit) { 
                        overturn = 1;
                    }

                    // std::cout << "  > initialize imu yaw" << std::endl;
                    outfile << "  > initialize imu yaw\n";
                    
                    yaw_init = yaw;

                    InitializeYaw();
                    PubTargetFlag(1);
                }
                else if(cnt_ready == 25 ) { // 
                    // std::cout << "  > mpc" << std::endl;
                    outfile << "  > mpc\n";
                    MPCMode();                   
                }
                else if(25 < cnt_ready && cnt_ready < 55) {
                    if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || cur_z_world > height_limit) { 
                        overturn = 1;
                    }
                }
                else if(cnt_ready == 55) {
                    // std::cout << "Tracking Step" << std::endl;
                    outfile << "  > Tracking Step\n";
                    step = 1;
                }
                // Use if you want to compare global elevation map with elevation map using realtime sensor 
                // else if(cnt_ready == 55) { //
                //     ResetElevationMap(); 
                //      if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || cur_z_world > height_limit) { 
                //         overturn = 1;
                //     }
                // }
                // else if(cnt_ready == 60) { // 
                //     std::cout << "Tracking Step" << std::endl;
                //     outfile << "  > Tracking Step\n";
                //     elevation_map_raw = elevation_map_raw_copy;
                //     step = 1;
                // }
            }

        }
        
        else if(step == 1) {
            if (get_isSuccess == 0) {
                
                cnt_duration++;
                cnt_path++;

                PublishPath(cnt_path);

                if(cnt_path == 1) {
                    std::cout << robot_namespace << "/" << data_id << "/" << yaw_init << "/" << global_x_tar << "/" << global_y_tar << std::endl;
                    // std::cout << "  > yaw_init: " << yaw_init << std::endl;
                    outfile << "  > yaw_init: " << yaw_init << "\n";
                    // std::cout << "  > global_x_tar: " << global_x_tar << std::endl;
                    outfile << "  > global_x_tar: " << global_x_tar << "\n";
                    // std::cout << "  > global_y_tar: " << global_y_tar << std::endl;
                    outfile << "  > global_y_tar: " << global_y_tar << "\n";
                }
                
                // print_check(cnt_duration, "  > moving ...");

                if(abs(yaw_target - _yaw_target) < 0.01) {
                    cnt_forward++;
                    if (cnt_forward == 20) {
                        PublishVelocity(0.05);
                        PubTargetFlag(3);
                    }
                }

                if(isArrive == 0) {
                    dis = distance(0,0,global_x_tar, global_y_tar, global_x_cur, global_y_cur);

                    if(abs(roll) > roll_limit || abs(pitch) > pitch_limit || dis > 0.1) { 
                        overturn = 1;
                    }

                    d = sqrt((global_x_tar - global_x_cur)*(global_x_tar - global_x_cur) + (global_y_tar - global_y_cur)*(global_y_tar - global_y_cur));
                
                    if ( d < R_success) { // arrive  
                        isArrive = 1;
                    }
                    else { //not arrive
                        if (overturn == 1) { // not arrive -> fail
                            get_isSuccess = 1;
                            isSuccess = 0;
                        }
                        else if(cnt_duration == 150) { // not arrive -> fail
                            get_isSuccess = 1;
                            isSuccess = 0;  
                        }
                        else { // not arrive -> wait
                            get_isSuccess = 0;        
                        }
                    }
                }
                else if(isArrive == 1) {
                    
                    if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
                        overturn = 1;
                    }

                    cnt_arrive++;
                    
                    if(cnt_arrive == 10 && overturn == 0) {
                        get_isSuccess = 1;
                        isSuccess = 1;
                    }
                    else if(cnt_arrive == 10 && overturn == 1) {
                        get_isSuccess = 1;
                        isSuccess = 0;
                    }
                }
            }

            else if (get_isSuccess == 1) {
                cnt_data++;

                if (cnt_data == 1) {

                    dataset.id = data_id;
                    dataset.world_x_init = x_init;
                    dataset.world_y_init = y_init;
                    dataset.world_z_init = z_init;
                    dataset.yaw_init = yaw_init;
                    dataset.global_x_tar = global_x_tar;
                    dataset.global_y_tar = global_y_tar;
                    dataset.duration = cnt_duration/10;
                    dataset.isSuccess = isSuccess;
                    dataset.elevation_map_realtime = elevation_map_raw;
                    
                    pub_dataset.publish(dataset);
                    
                    if (isSuccess == 1) {
                        // std::cout << "  > duration: " << dataset.duration << std::endl;
                        outfile << "  > duration: " << dataset.duration << "\n";   
                        // std::cout << "  > result: success" << std::endl;
                        outfile << "  > result: success\n";
                    }
                    else {
                        // std::cout << "  > result: failure" << std::endl;
                        outfile << "  > result: failure\n";
                    }

                }
                else if (cnt_data > 1) {
                    cnt_respawn++;
                    if (cnt_respawn == 1) { // 2s
                        // std::cout << "Respawn Step" << std::endl;
                        outfile << "Respawn Step\n";
                        // std::cout << "  > crawl" << std::endl;
                        outfile << "  > crawl\n";
                        CrawlMode();
                        PublishZeroPath();
                        PublishVelocity(0);
                    }
                    else if (cnt_respawn == 20) {
                        // std::cout << "  > respawn" << std::endl;
                        outfile << "  > respawn\n";
                        // std::cout << "=========================================" << std::endl;  
                        outfile << "=========================================\n";
                        Respawn();
                    }
                    else if (cnt_respawn == 40) {
                        data_id++;
                        
                        step = 0;
                        
                        get_isSuccess = false;
                        overturn = false;
                        isArrive = false;
                        
                        cnt_ready = 0;
                        cnt_path = 0;
                        cnt_forward = 0;
                        cnt_data = 0;
                        cnt_respawn = 0;
                        cnt_duration = 0;
                        cnt_resetsim = 0;
                        cnt_arrive = 0;


                    }   
                }  
            }
        }


        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}