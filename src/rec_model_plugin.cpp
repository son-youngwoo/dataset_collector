#include "ros/ros.h"

#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh> 
#include <gazebo/common/common.hh>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>

using namespace std;

//dummy nodehandle
ros::NodeHandle nh;  

namespace gazebo
{   
  class rec_model_plugin : public ModelPlugin
  {
    // subscriber
    private: ros::Subscriber sub_torque;

    // // publisher
    private: ros::Publisher pub_bodypose;
    // private: ros::Publisher pub_jointpos;
    // private: ros::Publisher pub_jointvel;
    // private: ros::Publisher pub_bodypos;
    // private: ros::Publisher pub_bodyvel;
    // private: ros::Publisher pub_imu2;
    // private: ros::Publisher pub_contact;
    
    private: ros::NodeHandle* node;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the joint controller
    private: physics::JointControllerPtr joint1_Controller_;
    private: physics::JointControllerPtr joint2_Controller_;
    private: physics::JointControllerPtr joint3_Controller_;
    private: physics::JointControllerPtr joint4_Controller_;
    private: physics::JointControllerPtr joint5_Controller_;
    private: physics::JointControllerPtr joint6_Controller_;
    private: physics::JointControllerPtr joint7_Controller_;
    private: physics::JointControllerPtr joint8_Controller_;
    private: physics::JointControllerPtr joint9_Controller_;
    private: physics::JointControllerPtr joint10_Controller_;
    private: physics::JointControllerPtr joint11_Controller_;
    private: physics::JointControllerPtr joint12_Controller_;
    private: physics::JointControllerPtr joint13_Controller_;

    // Pointer to the joint 
    private: physics::JointPtr joint1_;
    private: physics::JointPtr joint2_;
    private: physics::JointPtr joint3_;
    private: physics::JointPtr joint4_;
    private: physics::JointPtr joint5_;
    private: physics::JointPtr joint6_;
    private: physics::JointPtr joint7_;
    private: physics::JointPtr joint8_;
    private: physics::JointPtr joint9_;
    private: physics::JointPtr joint10_;
    private: physics::JointPtr joint11_;
    private: physics::JointPtr joint12_;
    private: physics::JointPtr joint13_;
    private: physics::LinkPtr link1_;
    private: physics::LinkPtr plane1_;

    // Pointer to the body
    private: physics::LinkPtr globalposition;
    // private: physics::LinkPtr imusensor;

    public: bool stateflag = 1;

    // Pointer to the update event connection
    private: std::vector<event::ConnectionPtr> updateConnection;


    public: rec_model_plugin()
    {
      // Start up ROS
      std::string name = "rec_model_plugin";
      int argc = 0;
      ros::init(argc, NULL, name);
    }

    public: ~rec_model_plugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      ROS_INFO("rec_model_plugin run");
      std::cerr << "\nThe rectangular model plugin is attach to model ["
                << this->model->GetName() << "]\n";

      std::string robot_namespace = "/"+this->model->GetName()+"/";

      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

      this->joint1_= this->model->GetJoint("joint1");
      
      // Store the joint Controller to control Joint
      this->joint1_Controller_= this->model->GetJointController();

      //subscriber
      this->sub_torque = this->node->subscribe<std_msgs::Float32>(robot_namespace+"Torque_sim", 100, &rec_model_plugin::ROSCallbackTorque_sim, this );

      // //publisher
      this->pub_bodypose = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"BodyPose_sim", 100);      
      // this->pub_jointpos = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"JointPos_sim", 100);
      // this->pub_jointvel = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"JointVel_sim", 100);
      // this->pub_bodypos = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"BodyPos_sim", 100);      
      // this->pub_bodyvel = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"BodyVel_sim", 100);   
      // this->pub_imu2 = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"IMU_sim2", 100); 
      // this->pub_contact = this->node->advertise<std_msgs::Float32MultiArray>(robot_namespace+"Contact_sim", 100);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&rec_model_plugin::OnUpdate, this)));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
        // get robot state from gazebo
        ignition::math::Pose3<double>  WorldPose = model->WorldPose(); 
        ignition::math::Vector3<double> Pos = WorldPose.Pos(); // body position
        ignition::math::Vector3<double> Rot_Euler = WorldPose.Rot().Euler(); // body euler angle position
        // // ignition::math::Vector3<double> WorldLinearAccel = model->WorldLinearAccel(); // body linear acceleration from world
        // ignition::math::Vector3<double> WorldLinearVel = model->WorldLinearVel(); // body linear velocity from world
        // ignition::math::Vector3<double> WorldAngularVel = model->WorldAngularVel(); // body angle velocity from world
        // ignition::math::Vector3<double> WorldAngularAccel = model->WorldAngularAccel(); // body euler angle acceleration from world
        // ignition::math::Vector3<double> RelativeAngularVel = model->RelativeAngularVel(); // body angular velocity from robot
        // ignition::math::Vector3<double> acc_rel = model->WorldLinearAccel(); 
        // ignition::math::Pose3<double>  model_pose_rel = model->RelativePose(); 
        // ignition::math::Vector3<double> rot_rel = model_pose_rel.Rot().Euler();

        // publish body position and euler angle
        std_msgs::Float32MultiArray BodyPose;
        BodyPose.data.clear();
        BodyPose.data.push_back(Pos.X());
        BodyPose.data.push_back(Pos.Y());
        BodyPose.data.push_back(Pos.Z());
        BodyPose.data.push_back(Rot_Euler.X());
        BodyPose.data.push_back(Rot_Euler.Y());
        BodyPose.data.push_back(Rot_Euler.Z());
        pub_bodypose.publish(BodyPose);

        // // publish joint position
        // std_msgs::Float32MultiArray JointPos;
        // JointPos.data.clear();
        // JointPos.data.push_back(this->joint1_->Position(1));
        // JointPos.data.push_back(this->joint2_->Position(1));
        // JointPos.data.push_back(this->joint3_->Position(1));
        // JointPos.data.push_back(this->joint4_->Position(1));
        // JointPos.data.push_back(this->joint5_->Position(1));
        // JointPos.data.push_back(this->joint6_->Position(1));
        // JointPos.data.push_back(this->joint7_->Position(1));
        // JointPos.data.push_back(this->joint8_->Position(1));
        // JointPos.data.push_back(this->joint9_->Position(1));
        // JointPos.data.push_back(this->joint10_->Position(1));
        // JointPos.data.push_back(this->joint11_->Position(1));
        // JointPos.data.push_back(this->joint12_->Position(1));
        // pub_jointpos.publish(JointPos);

        // // publish joint velocity
        // std_msgs::Float32MultiArray JointVel;
        // JointVel.data.clear();
        // JointVel.data.push_back(this->joint1_->GetVelocity(1));
        // JointVel.data.push_back(this->joint2_->GetVelocity(1));
        // JointVel.data.push_back(this->joint3_->GetVelocity(1));
        // JointVel.data.push_back(this->joint4_->GetVelocity(1));
        // JointVel.data.push_back(this->joint5_->GetVelocity(1));
        // JointVel.data.push_back(this->joint6_->GetVelocity(1));
        // JointVel.data.push_back(this->joint7_->GetVelocity(1));
        // JointVel.data.push_back(this->joint8_->GetVelocity(1));
        // JointVel.data.push_back(this->joint9_->GetVelocity(1));
        // JointVel.data.push_back(this->joint10_->GetVelocity(1));
        // JointVel.data.push_back(this->joint11_->GetVelocity(1));
        // JointVel.data.push_back(this->joint12_->GetVelocity(1));
        // pub_jointvel.publish(JointVel);

        // // publish body position
        // std_msgs::Float32MultiArray BodyPos;
        // BodyPos.data.clear();
        // BodyPos.data.push_back(Pos.X());
        // BodyPos.data.push_back(Pos.Y());
        // BodyPos.data.push_back(Pos.Z());
        // pub_bodypos.publish(BodyPos);

        // // publish body velocity
        // std_msgs::Float32MultiArray BodyVel;
        // BodyVel.data.clear();
        // BodyVel.data.push_back(WorldLinearVel.X());
        // BodyVel.data.push_back(WorldLinearVel.Y());
        // BodyVel.data.push_back(WorldLinearVel.Z());
        // pub_bodyvel.publish(BodyVel);

        // publish imu
        // std_msgs::Float32MultiArray IMU;
        // IMU.data.clear();
        // IMU.data.push_back(Rot_Euler.X()); // roll, pitch, yaw
        // IMU.data.push_back(Rot_Euler.Y());
        // IMU.data.push_back(Rot_Euler.Z());
        // IMU.data.push_back(RelativeAngularVel.X()); // roll, pitch, yaw velocity
        // IMU.data.push_back(RelativeAngularVel.Y());
        // IMU.data.push_back(RelativeAngularVel.Z()); 
        // IMU.data.push_back(WorldLinearAccel.X()); // roll, pitch, yaw acceleration
        // IMU.data.push_back(WorldLinearAccel.Y());
        // IMU.data.push_back(WorldLinearAccel.Z());
        // pub_imu2.publish(IMU);

        // get and publish contact states
        // std::string contact_link;
        // double LF_contactFlag = 0;
        // double RF_contactFlag = 0;
        // double LB_contactFlag = 0;
        // double RB_contactFlag = 0;

        // physics::ContactManager *contactManager = this->model->GetWorld()->Physics()->GetContactManager();
        // for(int i=0; i<contactManager->GetContactCount(); i++)
        // {
        //   physics::Contact *contact = contactManager->GetContact(i);
        //   contact_link = contact->collision1->GetLink()->GetName();
        //   if(contact_link == "LF_knee"){LF_contactFlag = 1;}
        //   if(contact_link == "RF_knee"){RF_contactFlag = 1;}
        //   if(contact_link == "RB_knee"){LB_contactFlag = 1;}
        //   if(contact_link == "LB_knee"){RB_contactFlag = 1;}
        // }

        // std_msgs::Float32MultiArray Contact;
        // Contact.data.clear();
        // Contact.data.push_back(LF_contactFlag);
        // Contact.data.push_back(RF_contactFlag);
        // Contact.data.push_back(LB_contactFlag);
        // Contact.data.push_back(RB_contactFlag);
        // pub_contact.publish(Contact);
      
      ros::spinOnce();
    }

    void ROSCallbackTorque_sim(const std_msgs::Float32::ConstPtr& torque)
    {
      this->joint1_->SetForce(0, torque->data);
    }
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(rec_model_plugin);

}
