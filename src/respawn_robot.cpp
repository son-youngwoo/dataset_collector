// #include "respawn_robot.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "respawn_robot");

//     ros::NodeHandle nh;

//     ROSInit(nh);


//     ros::Rate rate(2); // 1초에 2번

//     while(ros::ok()) {
//         if (respawn_flag == 0) {
//             timer0 += 0.5;
//             if(reset == 1) {
//                 ROS_ERROR("reset 0");
//                 reset = 0;
//                 timer0 = 0;
//                 respawn_flag = 5;
//             }

//             if(timer0 == 0.5) {
//                 InitializeSensor();
//             }
//             else if(timer0 == 6) {
//                 InitializeIMU();
//             }
//             else if(timer0 == 8) {
//                 CrawlMode();
                
//                 respawn_flag = 1 ;
//             }
//         }
//         else if (respawn_flag == 1) { // 1. mpc 켜는 명령 보내기, id 데이터셋에 저장하기.
//             timer1 += 0.5;
//             if(reset == 1) {
//                 ROS_ERROR("reset 1");
//                 reset = 0;
//                 timer1 = 0;
//                 respawn_flag = 5;
            
//             }
                
//             if(timer1 == 2) {
//                 InitializeIMU();
//             }
//             else if(timer1 == 4) {
//                 CrawlMode();
//             }
//             else if(timer1 == 5) {
//                 StandMode();
//             }
//             else if(timer1 == 6 ) {
//                 MPCMode();
//             }
//             else if(timer1 == 8) {
//                 ResetElevationMap();
//             }
//             else if(timer1 == 9) {
//                 timer1 = 0;

//                 dataset.id = data_id;

//                 respawn_flag = 2;    
//             }
//         }
//         else if (respawn_flag == 2) { // 2. elevation map 데이터셋에 저장하기.
//             if(reset == 1) {
//                 ROS_ERROR("reset 2");
//                 reset = 0;
//                 respawn_flag = 5;
            
//             }
//             GetElevationMap();
//         }
//         else if (respawn_flag == 3) {  // 3. 타겟 포지션 보내고 데이터셋에 저장하기.    
//             timer2 += 0.5;
//             timer_reset += 0.5;

//             if(reset == 1) {
//                 ROS_ERROR("reset 3");
//                 reset = 0;
//                 timer2 = 0;
//                 timer_reset = 0;
//                 respawn_flag = 5;
            
//             }
//             if(timer2 == 0.5) {       
//                 std::cout << "publish target position" << std::endl;        

//                 // Define the minimum and maximum radii
//                 double min_radius = 0.8;
//                 double max_radius = 1.0;
//                 double centerX = x;
//                 double centerY = y;

//                 x_init = x;
//                 y_init = y;
                
//                 // Define the random number generator
//                 std::random_device rd;
//                 std::mt19937 gen(rd());
//                 std::uniform_real_distribution<> dis_angle(0.0, 2.0 * M_PI);
//                 std::uniform_real_distribution<> dis_radius(min_radius, max_radius);
                
//                 // Generate a random angle and distance
//                 double angle = dis_angle(gen);
//                 double radius = dis_radius(gen);
                
//                 // Calculate the x and y coordinates
//                 rand_x_tar = centerX + radius * std::cos(angle); // world base
//                 rand_y_tar = centerY + radius * std::sin(angle); // world base

//                 // target x,y dataset 저장.
//                 dataset.x = rand_x_tar - centerX; // robot base
//                 dataset.y = rand_y_tar - centerY; // robot base

//                 yaw_target = atan2(rand_y_tar - y, rand_x_tar - x); // theta based world frame

//                 yaw_target_deg = abs(yaw_target/M_PI*180);

//                 num_div = yaw_target_deg / 6;
                
//                 yaw_target_dis = yaw_target / num_div;
//             }
//             std::cout << "move... timer_reset : " << timer_reset << std::endl;        
            
//             if(timer_reset >= 25) {
//                 timer_reset = 0;
//                 ROS_ERROR("reset");
            
//                 respawn_flag = 5;
//             }

//             k++;

//             if(k < num_div + 1) {
//                 _yaw_target = yaw_target_dis*k;
//             }
//             else {
//                 _yaw_target = yaw_target_dis*num_div;
//             }
        
//             tf2::Quaternion q;

//             q.setRPY(0, 0, _yaw_target);

//             nav_msgs::Path path;

//             path.header.stamp = ros::Time::now();
//             path.header.frame_id = "world";
//             path.poses.resize(2);  // allocate memory for the poses array
//             path.poses[0].header.frame_id = "world";
//             path.poses[0].header.stamp = ros::Time::now();
//             path.poses[0].pose.position.x = x_init;
//             path.poses[0].pose.position.y = y_init;
//             path.poses[0].pose.orientation.x = 0;
//             path.poses[0].pose.orientation.y = 0;
//             path.poses[0].pose.orientation.z = 0;
//             path.poses[0].pose.orientation.w = 1;

//             path.poses[1].header.frame_id = "world";
//             path.poses[1].header.stamp = ros::Time::now();
//             path.poses[1].pose.position.x = rand_x_tar;
//             path.poses[1].pose.position.y = rand_y_tar;
//             path.poses[1].pose.orientation.x = q.x();
//             path.poses[1].pose.orientation.y = q.y();
//             path.poses[1].pose.orientation.z = q.z();
//             path.poses[1].pose.orientation.w = q.w();

//             pub_path.publish(path);

//             if(abs(yaw_target - _yaw_target) < 0.01) {
                
//                 _timer2 += 0.5;

//                 if (_timer2 == 1) {
//                     k = 0;
//                     timer2 = 0;
//                     std_msgs::Float32 xvel_target;
//                     xvel_target.data = 0.1;
//                     pub_xvel.publish(xvel_target);

//                     respawn_flag = 4;
//                     _timer2 = 0;
//                 }

                
//             }

           

//         }
//         else if(respawn_flag == 4) { // 4. 성공인지 실패인지 결과가 나오면 데이터셋에 저장하고 퍼블리시하기.
            
//             timer_reset += 0.5;
//             if(reset == 1) {
//                 ROS_ERROR("reset 4");
//                 reset = 0;
//                 timer_reset = 0;
//                 respawn_flag = 5;
//             }
//             std::cout << "move... timer_reset : " << timer_reset << std::endl;        

//             if(timer_reset >= 25) {
//                 timer_reset = 0;
//                 ROS_ERROR("reset");
            
//                 respawn_flag = 5;
//             }

//             if (get_s_or_f == 1) {

//                 if (s_or_f == 1) {
//                     std::cout << "success !" << std::endl; 
//                 }
//                 else {
//                     std::cout << "fail !" << std::endl; 
//                 }
//                 dataset.s_or_f = s_or_f;

//                 pub_dataset.publish(dataset);
                
//                 respawn_flag = 5;
                
//                 timer_reset = 0;
//             }

//         }

        
//         else if(respawn_flag == 5) { // 5. 로봇 crawl 자세로 바꾸기
//             Stop();
//         }    

//         else if(respawn_flag == 6) { // 6. 랜덤 포지션으로 리스폰하기.
//             Respawn();

//             data_id++;
      
//             respawn_flag = 1;
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


    ros::Rate rate(2); // 1초에 2번

    while(ros::ok()) {  

        if (ov == 0) {
            if(abs(roll) > roll_limit || abs(pitch) > pitch_limit) {
                overturn = 1;
            }
            else {
                overturn = 0;
            }
        }

        d = sqrt((rand_x_tar - x)*(rand_x_tar - x) + (rand_y_tar - y)*(rand_y_tar - y));
        R_success = 0.15;

        if ( d < R_success) // arrive or not?
        {
            arrive = 1;
        }
        else {
            arrive = 0;
        }

        if(overturn  == 1 && arrive == 1) { // failure
            get_s_or_f = 1;
            s_or_f = 0;
        }
        else if(overturn == 1 && arrive == 0) { // reset      
            ov = 1;
            respawn_flag = 3;
        }
        else if(overturn == 0 && arrive == 1) { // success
            get_s_or_f = 1;
            s_or_f = 1;
        }
        else if(overturn == 0 && arrive == 0) { // wait result
            get_s_or_f = 0;
        }
        
        if (respawn_flag == 0) {
            
            timer0 += 0.5;
            
            if(timer0 == 0.5) {
                InitializeSensor();
            }
            else if(timer0 == 6) {
                respawn_flag = 1;
            }
            // else if(timer0 == 6) {
            //     InitializeIMU();
            // }
            // else if(timer0 == 8) {
            //     CrawlMode();
            //     respawn_flag = 1;
            // }
        }
        else if(respawn_flag == 1) {
            ROS_ERROR("%f", timer1);

            timer1 += 0.5;
            
            if(timer1 == 2) {
                InitializeIMU();
                ov = 0;
            }
            else if(timer1 == 4) {
                CrawlMode();
            }
            else if(timer1 == 5) {
                StandMode();
            }
            else if(timer1 == 6 ) {
                MPCMode();
            }
            else if(timer1 == 8) {
                ResetElevationMap();
            }
            else if(timer1 == 9) {
                dataset.id = data_id; // dataset id add
                GetElevationMap(); // dataset elevation add 
            }
            else if(timer1 == 10) {
                SetYawTarget();
            }
            else if(timer1 >= 11) {
                
                k++;

                if(k < num_div + 1) {
                    _yaw_target = yaw_target_dis*k;
                }
                else {
                    _yaw_target = yaw_target_dis*num_div;
                }

                tf2::Quaternion q;
                q.setRPY(0, 0, _yaw_target);

                tf2::Quaternion q_init;
                q_init.setRPY(roll, pitch, yaw);

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

                if(abs(yaw_target - _yaw_target) < 0.01) {
                    
                    timer2 += 0.5;

                    if (timer2 == 1) {
                        std_msgs::Float32 xvel_target;
                        xvel_target.data = 0.1;
                        pub_xvel.publish(xvel_target);

                        respawn_flag = 2;
                        // timer1 += 0.5 추가해야될듯!!
                    }
                }
            }
        }

        else if(respawn_flag == 2) { // 4. 성공인지 실패인지 결과가 나오면 데이터셋에 저장하고 퍼블리시하기.
            timer1 += 0.5;
            if(timer1 > 35) {
                respawn_flag = 3;
            }
            if (get_s_or_f == 1) {

                if (s_or_f == 1) {
                    std::cout << "success !" << std::endl; 
                }
                else {
                    std::cout << "fail !" << std::endl; 
                }
                dataset.s_or_f = s_or_f;

                pub_dataset.publish(dataset);
                
                respawn_flag = 3;      
            }

        }
        else if(respawn_flag == 3) { // 5. 로봇 crawl 자세로 바꾸기
            timer0 = 0;
            timer1 = 0;
            timer2 = 0;
            timer3 += 0.5;
            k = 0;

            arrive = 0;
            overturn = 0;

            if (timer3 == 0.5) {
                Stop();
            }
            else if (timer3 == 2.5) {
                Respawn();
            }
            else if (timer3 == 4.5) {
                if (ov == 0 ) {
                    data_id++;
                }
                timer3 = 0;
                respawn_flag = 0;
            }
            
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
