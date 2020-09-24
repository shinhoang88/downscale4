#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

//#include <kdl/chain.hpp>
//#include <kdl/chain.hpp>
//#include <kdl/chainfksolver.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "downscale4/one.h" 
#include "downscale4/two.h" 
#include "downscale4/three.h" 
#include "downscale4/seven.h" 

#define pi 3.14159265359

double joint1_actp, joint2_actp, joint3_actp, joint4_actp, joint5_actp, joint6_actp, joint7_actp;

double joint1_actv, joint2_actv, joint3_actv, joint4_actv, joint5_actv, joint6_actv, joint7_actv;

 ros::NodeHandle nh;  
////////////////////////// for quadruped robot ////////////////////////////////////////
ros::Publisher pubactp = nh.advertise<downscale4::seven>("downscale_actp", 100);    
ros::Publisher pubactv = nh.advertise<downscale4::seven>("downscale_actv", 100); 
ros::Publisher pubactt = nh.advertise<downscale4::seven>("downscale_actt", 100); 
//////////////////////////////////////////////////////////

namespace gazebo
{   
  class ROSdualdownscalePlugin : public ModelPlugin
  {

    public: ROSdualdownscalePlugin()
    {

      // Start up ROS
      std::string name = "ros_model_plugin_node1";
      int argc = 0;
      ros::init(argc, NULL, name);


    }
    public: ~ROSdualdownscalePlugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) //connect with gazebo
    {

      // Store the pointer to the model
      //this->model = _parent;		

      // ROS Nodehandle
      //this->node = new ros::NodeHandle("~");

	// Store the pointer to the model
      this->model = _parent;

      ROS_INFO("dualdownscale run");
      std::cerr << "\nThe dualdownscale plugin is attach to model ["
                << this->model->GetName() << "]\n";

      std::string robot_topic = "/"+this->model->GetName()+"/";

      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

/////////////////////////////////////////////////////for qua/////////////
	std::string model_joint;

      this->joint1_= this->model->GetJoint("J1"); // getting joint anle
      this->joint2_= this->model->GetJoint("J2");
      this->joint3_= this->model->GetJoint("J3");
      this->joint4_= this->model->GetJoint("J4");
      this->joint5_= this->model->GetJoint("J5");
      this->joint6_= this->model->GetJoint("J6");
      this->joint7_= this->model->GetJoint("J7");

      // Store the joint Controller to control Joint
      this->joint1_Controller_= this->model->GetJointController(); // in thw world file we have controller
      this->joint2_Controller_= this->model->GetJointController();
      this->joint3_Controller_= this->model->GetJointController();
      this->joint4_Controller_= this->model->GetJointController();
      this->joint5_Controller_= this->model->GetJointController();
      this->joint6_Controller_= this->model->GetJointController();
      this->joint7_Controller_= this->model->GetJointController();
////////////////////////////////////////////////////
   
//////////////////////sub for position and torque of quadruped/////////////////////////////////////////
 this->subp = this->node->subscribe<downscale4::seven>("downscale_jointp", 100, &ROSdualdownscalePlugin::ROSCallbackp, this );

 this->subt = this->node->subscribe<downscale4::seven>("downscale_jointt", 100, &ROSdualdownscalePlugin::ROSCallbackt, this );

////////////////////////////////////////////////////////////////////////////

      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSdualdownscalePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

    downscale4::seven actp;

	actp.a = this->joint1_->GetAngle(1).Radian();
	actp.b = this->joint2_->GetAngle(1).Radian();
	actp.c = this->joint3_->GetAngle(1).Radian();
	actp.d = this->joint4_->GetAngle(1).Radian();
	actp.e = this->joint5_->GetAngle(1).Radian();
	actp.f = this->joint6_->GetAngle(1).Radian();
	actp.g = this->joint7_->GetAngle(1).Radian();

         pubactp.publish(actp);

       downscale4::seven actv;

  actv.a = this->joint1_->GetVelocity(1);
	actv.b = this->joint2_->GetVelocity(1);
	actv.c = this->joint3_->GetVelocity(1);
	actv.d = this->joint4_->GetVelocity(1);
	actv.e = this->joint5_->GetVelocity(1);
	actv.f = this->joint6_->GetVelocity(1);
	actv.g = this->joint7_->GetVelocity(1);

  /*
	actv.a = this->joint1_->GetVelocity(1)*(180/pi);
	actv.b = this->joint2_->GetVelocity(1)*(180/pi);
	actv.c = this->joint3_->GetVelocity(1)*(180/pi);
	actv.d = this->joint4_->GetVelocity(1)*(180/pi);
	actv.e = this->joint5_->GetVelocity(1)*(180/pi);
	actv.f = this->joint6_->GetVelocity(1)*(180/pi);
	actv.g = this->joint7_->GetVelocity(1)*(180/pi);
  */
  /*
  actv.a = this->joint1_->GetVelocity(1)*(60/(2*pi))*160;
	actv.b = this->joint2_->GetVelocity(1)*(60/(2*pi))*160;
	actv.c = this->joint3_->GetVelocity(1)*(60/(2*pi))*160;
	actv.d = this->joint4_->GetVelocity(1)*(60/(2*pi))*160;
	actv.e = this->joint5_->GetVelocity(1)*(60/(2*pi))*160;
	actv.f = this->joint6_->GetVelocity(1)*(60/(2*pi))*160;
	actv.g = this->joint7_->GetVelocity(1)*(60/(2*pi))*160;
  */

       pubactv.publish(actv);

// get joint torque 

        physics::JointWrench joint1ft = this->joint1_->GetForceTorque(0); //force
	physics::JointWrench joint2ft = this->joint2_->GetForceTorque(0);
	physics::JointWrench joint3ft = this->joint3_->GetForceTorque(0);
	physics::JointWrench joint4ft = this->joint4_->GetForceTorque(0);
	physics::JointWrench joint5ft = this->joint5_->GetForceTorque(0);
	physics::JointWrench joint6ft = this->joint6_->GetForceTorque(0);
	physics::JointWrench joint7ft = this->joint7_->GetForceTorque(0);

  //gazebo::math::Vector3 j1torque = joint1ft.body2Torque; // torque
	//gazebo::math::Vector3 j2torque = joint2ft.body2Torque;
	//gazebo::math::Vector3 j3torque = joint3ft.body2Torque;
	//gazebo::math::Vector3 j4torque = joint4ft.body2Torque;
	//gazebo::math::Vector3 j5torque = joint5ft.body2Torque;
	//gazebo::math::Vector3 j6torque = joint6ft.body2Torque;
	//gazebo::math::Vector3 j7torque = joint7ft.body2Torque;

  
	gazebo::math::Vector3 j1torque = joint1ft.body2Force;
	gazebo::math::Vector3 j2torque = joint2ft.body2Force;
	gazebo::math::Vector3 j3torque = joint3ft.body2Force;
	gazebo::math::Vector3 j4torque = joint4ft.body2Force;
	gazebo::math::Vector3 j5torque = joint5ft.body2Force;
	gazebo::math::Vector3 j6torque = joint6ft.body2Force;
	gazebo::math::Vector3 j7torque = joint7ft.body2Force;

       downscale4::seven actt;

//	actt.a = j1torque.z;
//	actt.b = j2torque.y;
//	actt.c = j3torque.z;
//	actt.d = j4torque.y;
//	actt.e = j5torque.z;
//	actt.f = j6torque.y;
//	actt.g = j7torque.z;

        actt.a = j7torque.x; // x torque sensing // able to modify specific joint torque
       	actt.b = j7torque.y;
       	actt.c = j7torque.z;
       	actt.d = j3torque.x;
       	actt.e = j3torque.y;
       	actt.f = j3torque.x;
       	actt.g = j4torque.z;

       pubactt.publish(actt);

      ros::spinOnce();
    }
///////////////////////////////////////////////////////////////////////////////
void ROSCallbackp(const downscale4::seven::ConstPtr& position)
    { // when call it it will move joint position
      this->joint1_Controller_->SetJointPosition(joint1_, position->a, 0);
      this->joint2_Controller_->SetJointPosition(joint2_, position->b, 0);  
      this->joint3_Controller_->SetJointPosition(joint3_, position->c, 0);
      this->joint4_Controller_->SetJointPosition(joint4_, position->d, 0);  
      this->joint5_Controller_->SetJointPosition(joint5_, position->e, 0);
      this->joint6_Controller_->SetJointPosition(joint6_, position->f, 0);
      this->joint7_Controller_->SetJointPosition(joint7_, position->g, 0);
    }

void ROSCallbackt(const downscale4::seven::ConstPtr& torque)
    { // torque
      this->joint1_->SetForce(0, torque->a);
      this->joint2_->SetForce(0, torque->b);
      this->joint3_->SetForce(0, torque->c);
      this->joint4_->SetForce(0, torque->d);
      this->joint5_->SetForce(0, torque->e);
      this->joint6_->SetForce(0, torque->f);
      this->joint7_->SetForce(0, torque->g);
    }

/////////////////////////////////////////////////////////////////////////////

    // Pointer to the model
    private: physics::ModelPtr model;

// Pointer to the joint controller // tempamlate need to study! // innertia. coliolis 알아보기
    private: physics::JointControllerPtr joint1_Controller_;
    private: physics::JointControllerPtr joint2_Controller_;
    private: physics::JointControllerPtr joint3_Controller_;
    private: physics::JointControllerPtr joint4_Controller_;
    private: physics::JointControllerPtr joint5_Controller_;
    private: physics::JointControllerPtr joint6_Controller_;
    private: physics::JointControllerPtr joint7_Controller_;

  // Pointer to the joint // tempamlate
    private: physics::JointPtr joint1_;
    private: physics::JointPtr joint2_;
    private: physics::JointPtr joint3_;
    private: physics::JointPtr joint4_;
    private: physics::JointPtr joint5_;
    private: physics::JointPtr joint6_;
    private: physics::JointPtr joint7_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscriber
 ros::Subscriber subp;
 ros::Subscriber subt;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSdualdownscalePlugin)
}
