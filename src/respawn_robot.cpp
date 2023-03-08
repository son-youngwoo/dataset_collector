#include "respawn_robot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "respawn_robot");

    ros::NodeHandle nh;

    ROSInit(nh);


    ros::Rate rate(10); // 1초에 2번

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
        
        if (respawn_flag == 1) {
        
            cnt1++;
            
            if(cnt1 == 20) {
                InitializeSensor();
            }
            // else if(cnt0 == 70) {
            //     respawn_flag = 1;
            // }
        // }
        // else if(respawn_flag == 1) {

            // timer1 += 0.5;
            // cnt1++;
            
            if(cnt1 == 70) {
                InitializeIMU();
                ov = 0;
            }
            else if(cnt1 == 90) {
                CrawlMode();
            }
            else if(cnt1 == 110) {
                StandMode();
            }
            else if(cnt1 == 140 ) {
                MPCMode();
            }
            else if(cnt1 == 150) {
                ResetElevationMap();
            }
            else if(cnt1 == 160) {
                dataset.id = data_id; // dataset id add
                GetElevationMap(); // dataset elevation add 
            }
            else if(cnt1 == 170) {
                SetYawTarget();
            }
            else if(cnt1 >= 180) {
                
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
                    
                    cnt2 += 0.5;

                    if (cnt2 == 3) {
                        std_msgs::Float32 xvel_target;
                        xvel_target.data = 0.1;
                        pub_xvel.publish(xvel_target);

                        respawn_flag = 2;
                        // cnt1 += 0.5 추가해야될듯!!
                    }
                }
            }
        }

        else if(respawn_flag == 2) { // 4. 성공인지 실패인지 결과가 나오면 데이터셋에 저장하고 퍼블리시하기.
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
            cnt1 = 0;
            timer2 = 0;
            timer3 += 0.5;
            k = 0;
            cnt1 = 0;
            cnt0 = 0;
            cnt2 = 0;
            cnt3++;

            arrive = 0;
            overturn = 0;

            if (cnt3 == 5) {
                Stop();
            }
            else if (cnt3 == 25) {
                Respawn();
            }
            else if (cnt3 == 45) {
                if (ov == 0 ) {
                    data_id++;
                }
                cnt3 = 0;
                respawn_flag = 1;
            }
            
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


// if (respawn_flag == 0) {
            
//             timer0 += 0.5;
            
//             if(timer0 == 0.5) {
//                 InitializeSensor();
//             }
//             else if(timer0 == 6) {
//                 respawn_flag = 1;
//             }
//             // else if(timer0 == 6) {
//             //     InitializeIMU();
//             // }
//             // else if(timer0 == 8) {
//             //     CrawlMode();
//             //     respawn_flag = 1;
//             // }
//         }
//         else if(respawn_flag == 1) {
//             ROS_ERROR("%f", timer1);

//             timer1 += 0.5;
            
//             if(timer1 == 2) {
//                 InitializeIMU();
//                 ov = 0;
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
//                 dataset.id = data_id; // dataset id add
//                 GetElevationMap(); // dataset elevation add 
//             }
//             else if(timer1 == 10) {
//                 SetYawTarget();
//             }
//             else if(timer1 >= 11) {