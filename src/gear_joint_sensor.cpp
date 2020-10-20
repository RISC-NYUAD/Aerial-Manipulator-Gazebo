/*
   Created by: Dimitris Chaikalis, dimitris.chaikalis@nyu.edu
   Simple plugin, publishing the state of the Landing Gear motors
*/


#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <string>
#include <time.h>


namespace gazebo
{
class JointStateSensor : public ModelPlugin
{

public: ros::NodeHandle nh;
public: std::string namespc;
public: ros::Publisher  joint_state_pub;
public: double time;
public: uint32_t sequence;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
    	this->model = _parent;
    	bool False = 0;
    	this->model->SetSelfCollide(False);
		if(_sdf->HasElement("nameSpace"))
			namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
			
		sequence = 0;
	
		std::string topicName = namespc + "/joint_info" ;

		joint_state_pub = nh.advertise<sensor_msgs::JointState>(topicName, 1); 
	
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointStateSensor::onUpdate, this));
		

	}  

public:	void onUpdate()
	{
		
		// Get simulation time and initialize sensor message header //
		sequence++;
		ros::Time curr_time = ros::Time::now();
		
		std_msgs::Header header;	
		header.seq = sequence;
		header.frame_id = " " ;
		header.stamp = curr_time ;
		
		sensor_msgs::JointState joint_msg;
		joint_msg.header = header;
		
		std::vector< std::string > names ;
		std::vector< double > angles, velocities, efforts ;

		// Get pointers to relevant joints and iterate //
		physics::JointPtr joint;
		std::string joint_name;
		std::string standard_prefix = namespc + "::" + namespc + "/land" ;
				
		for(int i=1; i<3 ; i++)
		{
			joint_name = standard_prefix + std::to_string(i) + "_joint" ;
			joint = this->model->GetJoint(joint_name);
			double speed = joint->GetVelocity(0) ;
			double angle = joint->GetAngle(0).Radian() ;
			names.push_back(joint_name) ;
			angles.push_back(angle) ;
			velocities.push_back(speed) ; 
			efforts.push_back(0.0); 	
		}
		
		// Finalize building the sensor message and publish //
		joint_msg.name = names ;
		joint_msg.position = angles ;
		joint_msg.velocity = velocities ;
		joint_msg.effort = efforts ;
		
		joint_state_pub.publish(joint_msg);

	}



};
GZ_REGISTER_MODEL_PLUGIN(JointStateSensor)
}
