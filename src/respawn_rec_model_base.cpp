#include "respawn_rec_model.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "respawn_rec_model");

    ros::NodeHandle nh;

    ROSInit(nh);

    robot_namespace = argv[1]; // get robot_namespace from .launch 
        
    nh.getParam("/" + robot_namespace + "/respawn_rec_model_base/map_x", map_x);
    nh.getParam("/" + robot_namespace + "/respawn_rec_model_base/map_y", map_y);


    ofstream outfile("/home/son/Desktop/dataset/dataset4/terminal_output_" + robot_namespace + ".txt");

    ros::Rate rate(10); 
    
    while(ros::ok()) {
        cnt++;
        
        if(cnt == 1) {
            getRandomPos(rand_x_init, rand_y_init);
            
            std::cout << robot_namespace << " : " << id << std::endl;

            // std::cout << "\n" << std::endl;
            // std::cout << "====================" << id << "====================" << std::endl;
            // std::cout << "initialize robot" << std::endl;
            // std::cout << " > x : " << rand_x_init << std::endl;
            // std::cout << " > y : " << rand_y_init << std::endl;
            outfile << "\n";
            outfile << "====================" << id << "====================" << "\n";
            outfile << "initialize robot" << "\n";
            outfile << " > x : " << rand_x_init << "\n";
            outfile << " > y : " << rand_y_init << "\n";
        }

        if(flag_init == 1) {
            torque = kp*(base_yaw - yaw) + kd*(0 - yaw_vel);
            Rotate(torque); 
        }

        // if(flag_respawn == 1) {
        //     cnt_respawn++;

        //     if(cnt_respawn == 40) {
        //         step = 2;
        //         cnt_respawn = 0;
        //         flag_respawn = 0;
        //     }
                    
        // }
        if(flag_respawn == 2) {
            cnt_respawn++;

            if(cnt_respawn == 40) {
                step = 3;
                cnt_respawn = 0;
                flag_respawn = 0;
            }

        }

        // if(step == 1 && flag_respawn == 0) {

        //     cnt1++;

        //     if(cnt1 == 1) {    
        //         yaw_init = 0;
        //         Respawn(rand_x_init, rand_y_init, yaw_init);

        //         // std::cout << "0 deg checking start" << std::endl;
        //         outfile << "0 deg checking start" << "\n";
        //     } 

        //     else if(cnt1 > 10 && cnt1 <= 60) {
        //         if (cur_z_world > height_limit || abs(roll) > roll_limit || abs(pitch) > pitch_limit) {
        //             Respawn(0,-0.5,0);
                    
        //             flag_turn1 = 0;
        //             flag_respawn = 1;
        //             flag_init = 1;

        //             // std::cout << " > robot can't be here" << std::endl;
        //             outfile << " > robot can't be here" << "\n";
        //         }
        //         else {
        //             flag_init = 0;
        //             Rotate(100);
        //         } 
                
        //         if(cnt1 == 60) {
        //             left_yaw_max1 = yaw;

        //             // std::cout << " > left_yaw_max1 : " << left_yaw_max1/DEG2RAD << std::endl;
        //             outfile << " > left_yaw_max1 : " << left_yaw_max1/DEG2RAD << "\n";
        //         }

        //     }
        //     else if(cnt1 > 60 && cnt1 <= 110) {
        //         Rotate(-100);

        //         if(cnt1 == 110) {
        //             right_yaw_max1 = yaw;
        //             flag_init = 1;

        //             // std::cout << " > right_yaw_max1 : " << right_yaw_max1/DEG2RAD << std::endl;
        //             outfile << " > right_yaw_max1 : " << right_yaw_max1/DEG2RAD << "\n";
        //         }
        //     }
        //     else if(cnt1 > 110 && cnt1 <= 160){

        //     }
        //     else if(cnt1 == 161) {
        //         flag_turn1 = 1;
        //         step = 2;
        //     }
        // }

        if(step == 2 && flag_respawn == 0) {

            cnt2++;

            if (cnt2 == 1) {
                yaw_init = 90;
                Respawn(rand_x_init, rand_y_init, yaw_init);

                // std::cout << "90 deg checking start" << std::endl;
                outfile << "90 deg checking start" << "\n";
            }
            else if(cnt2 > 10 && cnt2 <= 60) {
                
                if (cur_z_world > height_limit || abs(roll) > roll_limit || abs(pitch) > pitch_limit) {
                    Respawn(0,-0.5,0);
                    
                    flag_turn2 = 0;
                    flag_respawn = 2;
                    flag_init = 1;

                    // std::cout << " > robot can't be here" << std::endl;
                    outfile << " > robot can't be here" << "\n";
                }
                else {
                    flag_init = 0;
                    Rotate(50);
                }
                
                if(cnt2 == 60) {
                               
                    left_yaw_max2 = yaw;

                    // std::cout << " > left_yaw_max2 : " << left_yaw_max2/DEG2RAD << std::endl;
                    outfile << " > left_yaw_max2 : " << left_yaw_max2/DEG2RAD << "\n";
                }
                

            }
            else if(cnt2 > 60 && cnt2 <= 110) {
                Rotate(-50);

                if(cnt2 == 110) {
                    right_yaw_max2 = yaw;
                    flag_init = 1;

                    // std::cout << " > right_yaw_max2 : " << right_yaw_max2/DEG2RAD << std::endl;
                    outfile << " > right_yaw_max2 : " << -right_yaw_max2/DEG2RAD << "\n";
                }
            }
            else if(cnt2 > 110 && cnt2 <= 160){
            }
            else if(cnt2 == 161) {
                flag_turn2 = 1;
                step = 3;
            }
        }

        else if(step == 3 && flag_respawn == 0) {
            // std::cout << "get rotatable area" << std::endl;
            outfile << "get rotatable area" << "\n";

            left_yaw_max2_deg = int(left_yaw_max2/DEG2RAD);
            right_yaw_max2_deg = int(right_yaw_max2/DEG2RAD);

            if(flag_turn2 == 0) {
                // dataset_rec_model.left_yaw_max = 300;
                // dataset_rec_model.right_yaw_max = 300;
                dataset_rec_model.left_yaw_max = 190;
                dataset_rec_model.right_yaw_max = 190;

                // std::cout << "* rotatable area : X" << std::endl;
                outfile << "* rotatable area : X" << "\n";
            }
            else if(flag_turn2 == 1) { 
                
                if(yaw_init == 0) {     
                    dataset_rec_model.left_yaw_max = left_yaw_max2_deg;
                    dataset_rec_model.right_yaw_max = -right_yaw_max2_deg;
                }
                else if(yaw_init == 90) {
                    // left_yaw_max
                    if (90 <= left_yaw_max2_deg && left_yaw_max2_deg <= 180) {
                        dataset_rec_model.left_yaw_max = left_yaw_max2_deg - 90;
                    }
                    else if (-180 <= left_yaw_max2_deg && left_yaw_max2_deg <= -90) {
                        dataset_rec_model.left_yaw_max = left_yaw_max2_deg + 270;
                    }                   

                    // right_yaw_max
                    dataset_rec_model.right_yaw_max = -right_yaw_max2_deg + 90;        
                }
                                
                                
                // std::cout << "* left max : " << dataset_rec_model.left_yaw_max << ", right max : " << dataset_rec_model.right_yaw_max << std::endl;
                outfile << "* left max : " << dataset_rec_model.left_yaw_max << ", right max : " << dataset_rec_model.right_yaw_max << "\n";


                // dataset_rec_model.left_yaw_max = int(left_yaw_max2_deg);
                // dataset_rec_model.right_yaw_max = int(right_yaw_max2_deg);



                // // std::cout << "* left max : " << int(left_yaw_max2_deg) << ", right max : " << int(right_yaw_max2_deg) << std::endl;
                // outfile << "* left max : " << int(left_yaw_max2_deg) << ", right max : " << int(right_yaw_max2_deg) << "\n";
                // // outfile << "* rotatable area : " << right_yaw_max2_deg << " -> " << left_yaw_max2_deg << "\n";
            }

            dataset_rec_model.id = id;
            dataset_rec_model.world_x_init = rand_x_init - map_x;
            dataset_rec_model.world_y_init = rand_y_init - map_y;

            pub_dataset_rec_model.publish(dataset_rec_model);
            
            id++;
            step = 2;
            cnt = 0;
            cnt2 = 0;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}