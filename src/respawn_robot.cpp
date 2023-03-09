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

#include "respawn_robot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "respawn_robot");

    ros::NodeHandle nh;

    ROSInit(nh);

    ros::Rate rate(10); 

    while(ros::ok()) {  

        if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) { 
            overturn = 1;
        }

        if (step == 2) {
            cnt1++;
            if(cnt1 == 10) {
                InitializeSensor();
            }
            else if(cnt1 == 70) {
                step = 0;
            }
        }
    
        else if (step == 0) { // preparing ...
            if(overturn == 1) { // respawn
                cnt_respawn++;
                
                if (cnt_respawn == 1) { // 0.1s
                    ROS_ERROR("imergency respawn");
                    CrawlMode();
                }
                else if (cnt_respawn == 20) { // 2s
                    Respawn();
                }
                else if (cnt_respawn == 40) { // 4s
                    cnt_ready = 0;
                    cnt_s_or_f = 0;
                    cnt_path = 0;
                    cnt_vel = 0;
                    cnt_data = 0;
                    cnt_respawn = 0;
                    get_s_or_f = 0;
                    overturn = 0;
                    step = 0;
                    std::cout << "=======================================" << std::endl;  
                }
            }
            else if (overturn == 0) {
                cnt_ready++;
                if(cnt_ready == 1) {
                    std::cout << "===================" << data_id << "===================" << std::endl;
                    std::cout << "Ready" << std::endl;
                    std::cout << "  > start" << std::endl;
                    StartMode();
                }
                else if(cnt_ready == 20) { // 
                    std::cout << "  > publish zero torque" << std::endl;
                    yaw_init = yaw;
                    PubZeroTorqueFlag(1);
                }
                else if(cnt_ready == 25) { // 2
                    std::cout << "  > initialize imu yaw" << std::endl;
                    InitializeYaw();  
                }
                else if(cnt_ready == 45) { //
                    std::cout << "  > publish torque"   << std::endl;  
                    PubZeroTorqueFlag(0); 
                }
                else if(cnt_ready == 50) { //
                    std::cout << "  > crawl"   << std::endl;  
                    CrawlMode();
                }
                else if(cnt_ready == 70) { // 
                    std::cout << "  > stand"  << std::endl;
                    StandMode();
                }
                else if(cnt_ready == 90 ) { //
                    if(z < 0.4) {
                        overturn = 1;
                    } 
                    std::cout << "  > mpc" << std::endl;
                    MPCMode();
                }
                else if(cnt_ready == 100) { // 
                    ResetElevationMap(); 
                }
                else if(cnt_ready == 105) { // 
                    std::cout << "Go" << std::endl;
                    elevation_map_raw = elevation_map_raw_copy;
                    step = 1;
                }
            }

        }
        
        else if(step == 1) {
            if (get_s_or_f == 0) {
                cnt_s_or_f++;

                d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
                R_success = 0.15;

                if ( d < R_success) // arrive
                {   
                    if (overturn == 1 || z < 0.3) { // arrive -> fail
                        get_s_or_f = 1;
                        s_or_f = 0;
                    }
                    else { // arrive -> success
                        get_s_or_f = 1;
                        s_or_f = 1;
                    }

                }
                else { //not arrive
                    if (overturn == 1) { // not arrive -> fail
                        get_s_or_f = 1;
                        s_or_f = 0;
                    }
                    else if(cnt_s_or_f == 150) { // not arrive -> fail
                        get_s_or_f = 1;
                        s_or_f = 0;  
                    }
                    else { // not arrive -> wait
                        get_s_or_f = 0;        
                    }
                }
          
                print_check(cnt_s_or_f, "  > moving ...");

                cnt_path++;
                PublishPath(cnt_path);
                
                if(abs(yaw_target - _yaw_target) < 0.01) {
                    cnt_vel++;
                    if (cnt_vel == 10) {
                        PublishVelocity();
                    }
                }
            }

            else if (get_s_or_f == 1) {
                cnt_data++;

                if (cnt_data == 1) {
                    dataset.id = data_id; // dataset id add
                    dataset.elevation_map_raw = elevation_map_raw;
                    dataset.global_initial_x = x_init; // world base
                    dataset.global_initial_y = y_init; // world base
                    dataset.local_target_x = rand_x_tar - x_init; // robot base
                    dataset.local_target_y = rand_y_tar - y_init; // robot base
                    dataset.s_or_f = s_or_f;

                    pub_dataset.publish(dataset);
                    
                    if (s_or_f == 1) {
                        std::cout << "  > result: success" << std::endl;
                    }
                    else {
                        std::cout << "  > result: failure" << std::endl;
                    }

                    std::cout << "  > publish dataset" << std::endl;
                }
                else if (cnt_data > 1) {
                    cnt_respawn++;

                    if (cnt_respawn == 1) {
                        std::cout << "Respawn Step" << std::endl;
                        std::cout << "  > crawl" << std::endl;
                        PublishZeroVelocity();
                        PublishZeroPath();
                        CrawlMode();
                    }
                    else if (cnt_respawn == 20) {
                        std::cout << "  > respawn" << std::endl;
                        Respawn();
                    }
                    else if (cnt_respawn == 40) {
                        data_id++;
                        cnt_ready = 0;
                        cnt_s_or_f = 0;
                        cnt_path = 0;
                        cnt_vel = 0;
                        cnt_data = 0;
                        cnt_respawn = 0;
                        get_s_or_f = 0;
                        overturn = 0;
                        step = 0;
                        std::cout << "=======================================" << std::endl;  
                    }   
                }  
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}