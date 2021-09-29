#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <mav_msgs/Actuators.h>
#include <iostream>
#include <random>
#include <fstream>
#include <Eigen/StdVector>
#include <string>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>


using namespace std; 	using Eigen::Matrix3d;
using Eigen::Matrix4f;	using Eigen::Vector3d;
using Eigen::Vector4d;  using Eigen::Quaterniond;
using Eigen::Vector2d;  using Eigen::Matrix2d;

typedef struct TypeForOdometry {
	Eigen::Vector3d position;
	Eigen::Vector3d orientation;
	Eigen::Quaterniond quatern;	
	Eigen::Vector3d velocity;
	Eigen::Vector3d angular_rate;
	Eigen::VectorXd robot_q;
	Eigen::VectorXd robot_qdot;
	Eigen::VectorXd gripper;
}DerivedPose;

namespace gazebo
{
class UASControlPlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: std::string linkName, namespc;
public: ros::Publisher  velocity_pub;
public: ros::Subscriber keyper_sub, kinova_sub, gear_sub, aruco_sub, bumper_sub;
public: double des_x,des_y,des_z,des_yaw ;
public: Eigen::VectorXd q_desired;
public: Eigen::VectorXd prev_error;
public: bool Land, ready_to_land;
public: double manipulator_bool, gear_bool;	
public: double mx_hat,my_hat,mz_hat, nx, ny, nz;
public: Eigen::Vector3d drone_pos ;
public: Eigen::Quaterniond drone_rot ;
public: Eigen::Vector3d prev_Perr;
public: double time,prev_time,fingers;
public: double take_off_complete, landing_gear_retracted, system_armed, robot_ready;
public: int aruco_counter ;
public: double land_time;
public: Eigen::Vector3d object;
public: int ticker ; 
public: physics::JointPtr gear1, gear2;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{

    	this->model = _parent;
		des_x = 0;	 des_y = 0;	 manipulator_bool = 0;
		des_z = 0;	 des_yaw = 0;
		fingers = 0;
		system_armed = 0;
		take_off_complete = 0;
		landing_gear_retracted = 0;
		robot_ready = 0;
		Land = false;
		ready_to_land = false;
		land_time = 0.0;
		ticker = 0;

		Eigen::VectorXd temp(7), tester(7);
		tester << 0,2.8,0.4,3.14,0.2,3.6,0.6 ;
		temp << 1.57,1.8,0,4.4,0,4.71,1.57 ;
		q_desired = temp;
		temp << 0,0,0,0,0,0,0 ;
		prev_error = temp ;
		prev_Perr << 0,0,0;
		
		aruco_counter = 0;
		object << 0, 0, 0;
		
		std::ofstream ufile;
		ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/UAV.txt");
		ufile << "TIME, x, y, z, roll, pitch, yaw \n";
		ufile.close();

		ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/ROBOT.txt");
		ufile << "TIME, q1, q2, q3, q4, q5, q6, q7, finger1, finger2, finger3 \n";
		ufile.close();

		ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/RANGE_FINDER.txt");
		ufile << "TIME, d \n";
		ufile.close();

		ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/ADAPTS.txt");
		ufile << "TIME, mhat, nxhat, nyhat, gx, gy, gz \n";
		ufile.close();

		
		if(_sdf->HasElement("linkName"))
			linkName = _sdf->GetElement("linkName")->Get<std::string>();
		if(_sdf->HasElement("nameSpace"))
			namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
		if(_sdf->HasElement("x_desired"))
			des_x  = _sdf->Get<double>("x_desired");
		if(_sdf->HasElement("y_desired"))
			des_y  = _sdf->Get<double>("y_desired");
		if(_sdf->HasElement("z_desired"))
			des_z  = _sdf->Get<double>("z_desired");
		if(_sdf->HasElement("yaw_desired"))
			des_yaw  = _sdf->Get<double>("yaw_desired");
		if(_sdf->HasElement("manipulator_boolean"))
			manipulator_bool  = _sdf->Get<double>("manipulator_boolean");
		if(_sdf->HasElement("with_landing_gear"))
			gear_bool  = _sdf->Get<double>("with_landing_gear");
		
		if(!gear_bool){
			landing_gear_retracted = 1;
		}
		
		if(manipulator_bool){
			mx_hat = 14.7+5.5;	my_hat = 14.7+5.5;	mz_hat = 14.7+5.5;
		}else{
			mx_hat = 14.7;	my_hat = 14.7;	mz_hat = 14.7;
		}
		nx = 0;	ny = 0; nz = 0;

		std::string mototopic = namespc + "/command/motor_speed";
		std::string keypTopic = namespc + "/teleoperator" ;
		std::string kinoTopic = namespc + "/kinovaOper" ;
		std::string gearTopic = namespc + "/joint_info";
		std::string arucoTopic = namespc + "/aruco_estimate" ;
		std::string bumperTopic = "/ranger" ; 
 
		velocity_pub = nh.advertise<mav_msgs::Actuators>(mototopic, 1);
		keyper_sub = nh.subscribe(keypTopic, 1, &UASControlPlugin::keyper_callback, this);	
		kinova_sub = nh.subscribe(kinoTopic, 1, &UASControlPlugin::kinova_callback, this);
		gear_sub = nh.subscribe(gearTopic, 1, &UASControlPlugin::gear_callback, this);
		aruco_sub = nh.subscribe(arucoTopic, 1, &UASControlPlugin::aruco_callback, this);
		bumper_sub = nh.subscribe(bumperTopic, 1, &UASControlPlugin::bumper_callback, this);
	
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UASControlPlugin::onUpdate, this));
		

	}  

public:	void onUpdate()
	{

		physics::WorldPtr world = this->model->GetWorld();
		time = world->GetSimTime().Double();
		if(time==0.001){
			prev_time = 0;
		}
		
		if(!system_armed){
			if(time>0.1){
				system_armed = 1;
				std::cout << "\nAll Systems Ready... \n   Initiating Take-Off" << std::endl;
				std::cout << "\nTeleoperation Services have been temporarily disabled" << std::endl;
			}
		}		
		
		
		// GET THE POINTER TO THE AERIAL VEHICLE BODY  //
		physics::LinkPtr  quad = this->model->GetLink(linkName);
		
		std::random_device rd{};
    	std::mt19937 gen{rd()};
    	std::normal_distribution<> noise_{0,0.03};
		
		
		std::string kinova_prefix = namespc + "::j2s7s300_joint_" ;
		std::string finger_prefix = namespc + "::j2s7s300_joint_finger_" ;
		std::string tip_prefix 	  = namespc + "::j2s7s300_joint_finger_tip_" ;
		
		gear1 = this->model->GetJoint(namespc + "::" + namespc + "/land1_joint") ;
		gear2 = this->model->GetJoint(namespc + "::" + namespc + "/land2_joint") ;
		
		DerivedPose foundpose;
		if(manipulator_bool){
			Eigen::VectorXd angs(7);
			Eigen::VectorXd angd(7);
			for(int i=1;i<8;i++){
				physics::JointPtr joint_i = this->model->GetJoint(kinova_prefix + std::to_string(i));
				double q = joint_i->GetAngle(0).Radian();
				double qd = joint_i->GetVelocity(0);
				angs(i-1) = q;
				angd(i-1) = qd;
			}
			Eigen::VectorXd fing(6);
			for(int i=1;i<4;i++){
				physics::JointPtr finger_i = this->model->GetJoint(finger_prefix + std::to_string(i));
				double q = finger_i->GetAngle(0).Radian();
				fing(i-1) = q;
			}
			for(int i=1;i<4;i++){
				physics::JointPtr finger_i = this->model->GetJoint(tip_prefix + std::to_string(i));
				double q = finger_i->GetAngle(0).Radian();
				fing(i+2) = q;
			}
			foundpose.gripper = fing;
			foundpose.robot_q = angs;
			foundpose.robot_qdot = angd;
		}		
		// GET THE POSITION, ORIENTATION, ANGULAR-VELOCITY AND LINEAR-VELOCITY OF THE QUADROTOR  //
		gazebo::math::Pose pose = quad->GetWorldPose();  		
		gazebo::math::Vector3 ang_vel = quad->GetRelativeAngularVel();	
		gazebo::math::Vector3 lin_vel = quad->GetRelativeLinearVel();			
		gazebo::math::Vector3 position = pose.pos;
		gazebo::math::Quaternion quatern = pose.rot;

		
		foundpose.position = Eigen::Vector3d(position.x , position.y , position.z);	

		Eigen::Quaterniond quat1(quatern.w, quatern.x, quatern.y, quatern.z);
		if(quat1.norm() < std::numeric_limits<double>::epsilon()) {
			quat1.setIdentity();
		} else {
			quat1.normalize();
		}
		foundpose.quatern = quat1;	

		if ( my_norm(quatern) < std::numeric_limits<double>::epsilon()) {
			quatern.SetToIdentity();
		} else {
			quatern.Normalize();
		}
		gazebo::math::Vector3 rpy = quatern.GetAsEuler();	// we need both versions of the quaternion (Eigen::Quaterniond and gazebo::math::Quaternion) and we need them normalized
									// the reason is that the Eigen one is optimal for control later, the gazebo one is foolproof for transforming to RPY
		foundpose.orientation << rpy.x + noise_(gen) , rpy.y + noise_(gen) , rpy.z + noise_(gen) ;	// found the RPY orientation

		foundpose.velocity = Eigen::Vector3d(lin_vel.x, lin_vel.y, lin_vel.z);	// found the linear velocity

		foundpose.angular_rate = Eigen::Vector3d(ang_vel.x, ang_vel.y, ang_vel.z);	// found the angular velocity

		drone_pos << position.x, position.y, position.z ;
		drone_rot = foundpose.quatern ;
		
		std::ofstream ufile;
	  	ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/UAV.txt", std::ios::out | std::ios::app);
	    ufile << time << ", " << position.x << ", " << position.y << ", " << position.z << ", " << foundpose.orientation(0) << ", " << foundpose.orientation(1) << ", " << foundpose.orientation(2) << "\n" ;
	    ufile.close();	

	  	ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/ROBOT.txt", std::ios::out | std::ios::app);
	    ufile << time << ", " << foundpose.robot_q(0) << ", " << foundpose.robot_q(1) << ", " << foundpose.robot_q(2) << ", " << foundpose.robot_q(3) << ", " << foundpose.robot_q(4) << ", " << foundpose.robot_q(5) << ", " << foundpose.robot_q(6) << ", " << foundpose.gripper(0) << ", " << foundpose.gripper(1) << ", " << foundpose.gripper(2) << "\n" ;
	    ufile.close();	

		
		if(take_off_complete && aruco_counter==-1 && !robot_ready){
			Eigen::VectorXd temp(7);
			temp << 1.57, 2.6725, 0.0, 4.749, 0.0, 2.616, 1.57;
			q_desired = temp ;
		}

		if(Land){
			Eigen::VectorXd temp(7);
			temp << 1.57,1.7,0,4.4,0,4.71,1.57 ;			
			q_desired = temp;
			if(ready_to_land){
				des_z = des_z - 0.0002 ; 
			}
		}
				
		Eigen::VectorXd torques(7);
		if(manipulator_bool){
			kinova_control(foundpose,&torques);
			for(int i=1;i<8;i++){
				physics::JointPtr joint_i = this->model->GetJoint(kinova_prefix + std::to_string(i));
				joint_i->SetForce(0,torques(i-1));
			}
	
			Eigen::VectorXd finger_torques(6);
			control_gripper(foundpose,&finger_torques);
			for(int i=1;i<4;i++){
				physics::JointPtr joint_i = this->model->GetJoint(finger_prefix + std::to_string(i));
				joint_i->SetForce(0,finger_torques(i-1));
			}
			for(int i=1;i<4;i++){
				physics::JointPtr joint_i = this->model->GetJoint(tip_prefix + std::to_string(i));
				joint_i->SetForce(0,finger_torques(i+2));
			}
		}		
		if(system_armed){
			calc_n_publish(foundpose, torques(0));	
		}
		prev_time = time;
		
	}



public: double my_norm( gazebo::math::Quaternion& quat )
	{
		double w = quat.w;	double x = quat.x;	double y = quat.y;	double z = quat.z;
		double norm = (w*w)+(x*x)+(y*y)+(z*z);
		norm = sqrt(norm);
		return norm;
	}
	

public: void control_gripper(DerivedPose foundpose,Eigen::VectorXd* finger_torques)
	{
		Eigen::VectorXd angles = foundpose.gripper;
		Eigen::VectorXd temp_tau(6);
		Eigen::VectorXd Kp(6);
		Kp << 2,2,2,1.5,1.5,1.5;
		
		for(int i=0;i<6;i++)
		{
			double error = fingers - angles(i);
			double torque = Kp(i)*error;
			temp_tau(i) = torque;
		}
		*finger_torques = temp_tau;
	}


public: void kinova_control(DerivedPose foundpose,Eigen::VectorXd* torques)
	{
		Eigen::VectorXd angles = foundpose.robot_q;
		Eigen::VectorXd angvel = foundpose.robot_qdot;
		Eigen::Vector3d orient = foundpose.orientation;
		Eigen::VectorXd temp_tau(7);
		
		if(!robot_ready && aruco_counter==-1 && take_off_complete && my_abs(angles(5)-q_desired(5))<2e-1 && my_abs(angvel(1))<3e-2 && my_abs(angvel(3))<3e-2 && my_abs(angvel(5))<3e-2){
			robot_ready = 1;
			Eigen::Vector3d tip ;
			tip << 0.0, 0.6, -0.41; 
			des_x = object[0]-tip[0];	des_y = object[1]-tip[1]-0.004;	 des_z = object[2]-tip[2]-0.094;
			std::cout << des_y << " , " << des_z << std::endl;
		}
		
		Eigen::VectorXd p_gains(7),d_gains(7),i_gains(7);
		p_gains << 3,140,17,160,28,180,10;
		d_gains << 1,20,0.26,14,0.17,7,0.1;
		i_gains << 5000,100000,5000,100000,4000,100000,0.1;
		
		
		for(int i=0;i<7;i++)
		{
			double error = q_desired(i) - angles(i);
			double torque = p_gains(i)*(error) + d_gains(i)*(0 - angvel(i)) + i_gains(i)*integral(error,prev_error(i));
			temp_tau(i) = torque;
			prev_error(i) = error;
		}
		*torques = temp_tau;
		
	}
	
	
public: double my_abs(double x)
	{
		if(x>0){
			return x;
		}else{
			return -1.0*x;
		}
	
	}
	
	
public: double integral(double error,double prev_error)
	{
		double dt = time-prev_time;
		double result = (prev_error + error)*dt/2;
		return result;
	}

public: void calc_n_publish(DerivedPose foundpose, double z_moment)
	{

		double desired[8]={des_x,0.0,des_y,0.0,des_z,0.0,des_yaw,0.0}; 		 
		Eigen::VectorXd rotor_velocities(8);
		computeQuadControl(foundpose, desired, z_moment, &rotor_velocities);	

		mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
		actuator_msg->angular_velocities.clear();

		
		for(int i=0;i<8;i++)
		{
			if(Land && foundpose.position(2) < 0.5){
				actuator_msg->angular_velocities.push_back(0.0);
			}else{
				actuator_msg->angular_velocities.push_back(rotor_velocities(i));
			}
		}
		
		
		velocity_pub.publish(actuator_msg);
		

	}
	

public: void computeQuadControl(DerivedPose foundpose, double desiredState[8], double yaw_mom, Eigen::VectorXd* rotor_velocities)
	{  
		/*
		Controller based on the paper: 
		Adaptive Control Approaches for an Unmanned Aerial Manipulation System, 2020 International Conference on Unmanned Aerial Systems, 498-503 
		by Dimitris Chaikalis, Anthony Tzes, Farshad Khorrami
		*/
		Eigen::Vector3d posit = foundpose.position;
		Eigen::Vector3d lin_vel = foundpose.velocity;
		Eigen::Vector3d ang_vel = foundpose.angular_rate;
		Eigen::Vector3d orient = foundpose.orientation;
		Eigen::Quaterniond quatern = foundpose.quatern;
		Eigen::VectorXd kinova_angles = foundpose.robot_q;

		//  state variables //
		double x,y,z,x_d,y_d,z_d,phi,theta,psi,phi_d,theta_d,psi_d;

		x=posit[0];		 y=posit[1];	   z=posit[2];

		Eigen::Matrix3d RotMat = quatern.toRotationMatrix();
		Eigen::Vector3d world_vel = RotMat * lin_vel ;
		x_d=world_vel[0];	 y_d=world_vel[1];   z_d=world_vel[2];

		phi=orient[0];   	theta=orient[1];  		psi=orient[2];
		phi_d=ang_vel[0];	theta_d=ang_vel[1];		psi_d=ang_vel[2];  // notice that the angular velocities are not corrected with RotMat, cause we have designed the system
																	   // in the body frame as far as orientation and angular velocity and acceleration are concerned.

		if(!take_off_complete){
			if( (z>1.2) && my_abs(z_d)<2e-2 ){
				take_off_complete = 1;
				std::cout << "\nTake Off Completed Successfully... \n " << std::endl;
				if(!landing_gear_retracted){
					std::cout << "Retracting Landing Gear \n " << std::endl;
				}
			}
		}


		//  desired state variables //
		double xdes, xddes, ydes, yddes, zdes, zddes, phides, phiddes, thetades, thetaddes, psides, psiddes;
		xdes = desiredState[0];	ydes = desiredState[2];	zdes = desiredState[4]; psides = desiredState[6];	

		xddes = desiredState[1];	yddes = desiredState[3];	zddes = desiredState[5];
		phiddes = 0;			    thetaddes = 0;				psiddes = desiredState[7];	

		//  constants //
		double m_manip;
		if(manipulator_bool)
			m_manip = 5.5;
		else
			m_manip = 0.0;
		double mass = 14.7 + 0.095 + m_manip;  // mass of each rotor is 0.01 plus mass of the IMU which is 0.015 
		double grav = 9.81;
		double Ixx=1.57;	double Iyy=3.93;	double Izz=2.59;
		double xlen = 0.53; double ylen = 0.57;

		//  GAINS //
		double Kpx=3;   double Kpy=3;   double Kpz=1.3;  

		double Kdx=0.7;   double Kdy=0.3;   double Kdz=3; 

		double Kpph=10;    double Kpth=10;    double Kpps=1; 

		double Kdph=26;  double Kdth=26;  double Kdps=2;

		double Kix=0.2;	double Kiy=0.2;	double Kiz=0.3;

/*		double constU1 = mass/(cos(phi)*cos(theta));
		double z7 = z - zdes;
		double p7 = integral(z7,prev_Perr(2));		
		double z8star = zddes - Kpz*z7;
		double z8 = z_d - z8star;
		double U1 =   constU1*(grav - Kpz*(z8 - Kpz*z7) - z7 - Kdz*z8);    


		double z9 = x - xdes;
		double z10star = xddes - Kpx*z9;
		double z10 = x_d - z10star;
		double ux = (mass/U1)*(-Kpx*(z10 - Kpx*z9) -z9 -Kdx*z10);
		
		double z11 = y - ydes;
		double z12star = yddes - Kpy*z11;
		double z12 = y_d - z12star;
		double uy = (mass/U1)*(-Kpy*(z12 - Kpx*z11) -z11 - Kdy*z12);
*/
		double e5 = zdes - z;
		double p5 = integral(e5,prev_Perr(2));
		double e6 = Kpz*e5 + zddes + Kiz*p5 - z_d;
		double cz1 = 3;	
		double mz_dot = cz1*e6*(grav + Kiz*e5 + Kpz*(-Kpz*e5 - Kiz*p5 + e6) + e5 + Kdz*e6);
		mz_hat = mz_hat + mz_dot*(time-prev_time); 
		double U1 =   (mz_hat/(cos(phi)*cos(theta)))*(grav + Kiz*e5 + Kpz*(-Kpz*e5 - Kiz*p5 + e6) + e5 + Kdz*e6 );    // found the desired thrust


		double e1 = xdes - x;
		double p1 = integral(e1,prev_Perr(0));
		double e2 = xddes + Kpx*e1 + Kix*p1 - x_d; 
		double cx1 = 2;		
		double mx_dot = cx1*e2*(Kix*e1 - Kpx*Kpx*e1 - Kix*Kpx*p1 + Kpx*e2 + e1 + Kdx*e2);
		mx_hat = mx_hat + mx_dot*(time-prev_time);
		double ux = (mx_hat/U1)*(Kix*e1 - Kpx*Kpx*e1 - Kix*Kpx*p1 + Kpx*e2 + e1 + Kdx*e2);
		
		double e3 = ydes - y;
		double p3 = integral(e3,prev_Perr(1));
		double e4 = yddes + Kpy*e3 + Kiy*p3 - y_d;
		double cy1 = 2;		
		double my_dot = cy1*e4*(Kiy*e3 - Kpy*Kpy*e3 - Kiy*Kpy*p3 + Kpy*e4 + e3 + Kdy*e4);
		my_hat = my_hat + my_dot*(time-prev_time);
		double uy = (my_hat/U1)*(Kiy*e3 - Kpy*Kpy*e3 - Kiy*Kpy*p3 + Kpy*e4 + e3 + Kdy*e4);


		double alpha = cos(psides);	double beta = sin(psides);
		double v1 = alpha*ux + beta*uy;
		double v2 = beta*ux - alpha*uy;
		double sphi = minn(maxx(v2,-1),1);	double cphi = sqrt(1 - pow(sphi,2));
		phides = atan2(sphi, cphi);
		v1 = v1/cos(phides);
		double stheta = minn(maxx(v1,-1),1); double ctheta = sqrt(1 - pow(stheta,2));
		thetades = atan2(stheta, ctheta); 
		
		Eigen::Vector3d tau_g;
		harrier_grav_feedback(orient, kinova_angles, &tau_g); 
		double z_mom = 0;
		if(manipulator_bool){
			z_mom = yaw_mom;
		}

		double z1 = phi - phides;
		double z2star = phiddes - Kpph*z1;
		double z2 = phi_d - z2star;
		double gamma_x = 3;
		double nx_dot = gamma_x*z2;
		nx = nx + nx_dot*(time-prev_time);
		double U2 = (Ixx/xlen)*( - Kpph*(z2 - Kpph*z1) - z1 - Kdph*z2 - nx - xlen*tau_g(0)/Ixx) + (1/xlen)*((Izz-Iyy)*theta_d*psi_d)  ;
		
		double z3 = theta - thetades;
		double z4star = thetaddes - Kpth*z3;
		double z4 = theta_d - z4star;
		double gamma_y = 3;
		double ny_dot = gamma_y*z4;
		ny = ny + ny_dot*(time-prev_time);
		double U3 = (Iyy/ylen)*( - Kpth*(z4 - Kdth*z3) - z3 - Kdth*z4 - ny - ylen*tau_g(1)/Iyy) + (1/ylen)*((Ixx-Izz)*phi_d*psi_d) ;
		
		double z5 = psi - psides;
		double z6star = psiddes - Kpps*z5;
		double z6 = psi_d - z6star;
		double U4 = Izz*( - Kpps*(z6 - Kdps*z5) - z5 - Kdps*z6 - tau_g(2)/Izz + z_mom/Izz) + (Iyy-Ixx)*phi_d*theta_d ;

		prev_Perr(0) = e1;	prev_Perr(1) = e3;	prev_Perr(2) = e5;


		// FINALIZATION AND EXITING //
		Eigen::Vector3d angular_acc;	
		angular_acc << U2 , U3 , U4 ;
		double thrust = U1;	

		Eigen::Vector4d angular_acceleration_thrust;
		angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acc;
		angular_acceleration_thrust(3) = thrust;
	
		Eigen::MatrixXd acc_to_sqvel_matrix(8,4);
		calculateMatrixM(&acc_to_sqvel_matrix);

		*rotor_velocities = acc_to_sqvel_matrix * angular_acceleration_thrust ;
		*rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(8));
		*rotor_velocities = rotor_velocities->cwiseSqrt();  
		
		std::ofstream ufile;
	  	ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/ADAPTS.txt", std::ios::out | std::ios::app);
	    ufile << time << ", " << mz_hat << ", " << nx << ", " << ny << ", " << tau_g(0) << ", " << tau_g(1) << ", " << tau_g(2)+yaw_mom << "\n" ;
		ufile.close();

	}
	
	
public: void harrier_grav_feedback(Eigen::Vector3d rpy, Eigen::VectorXd angles, Eigen::Vector3d* grav_terms)
	{
		Eigen::Vector3d tau_W;
		
		double q1 = angles(0);	double q2 = angles(1);	double q3 = angles(2);
		double q4 = angles(3);	double q5 = angles(4);	double q6 = angles(5);	// q7 is unnecessary
		double phi = rpy[0];		double theta = rpy[1];	double psi = rpy[2];
		
		tau_W(0) = cos(phi)*cos(psi)*1.612305434011315E-15+cos(q4)*(cos(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))+sin(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi)))*6.158789907116443-sin(q5)*(cos(q4)*(sin(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)+cos(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0))*1.0-sin(q4)*(cos(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))+sin(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))))*1.317188700014071E-4-cos(psi)*sin(phi)*1.316547297686338E+1-cos(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)*2.713306697987719E-1+sin(q4)*(sin(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)+cos(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0))*6.158789907116443+sin(q6)*(cos(q5)*(cos(q4)*(sin(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)+cos(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0))*1.0-sin(q4)*(cos(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))+sin(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))))-sin(q5)*(cos(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)*1.0-sin(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0))*1.0)*1.816157378576463+cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)*7.894146239959809E-2-cos(q5)*(cos(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)*1.0-sin(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0))*1.317188700014071E-4+sin(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0)*2.713306697987719E-1-cos(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))*1.383065284555778E+1-cos(q6)*(cos(q4)*(cos(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))+sin(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi)))+sin(q4)*(sin(q3)*(cos(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)-cos(theta)*sin(psi)*sin(q1)*1.0)+cos(q3)*(sin(q2)*(cos(phi)*cos(psi)*1.224646799141545E-16-cos(psi)*sin(phi)*1.0+sin(phi)*sin(psi)*sin(theta)*1.224646799141545E-16+cos(phi)*sin(psi)*sin(theta))-cos(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.0)))*1.816157378576463-sin(q2)*(sin(q1)*(cos(phi)*cos(psi)+cos(psi)*sin(phi)*1.224646799141545E-16+sin(phi)*sin(psi)*sin(theta)-cos(phi)*sin(psi)*sin(theta)*1.224646799141545E-16)+cos(q1)*cos(theta)*sin(psi))*1.383065284555778E+1+sin(phi)*sin(psi)*sin(theta)*1.612305434011315E-15+cos(phi)*sin(psi)*sin(theta)*1.316547297686338E+1-cos(theta)*sin(psi)*sin(q1)*7.894146239959809E-2; 
		tau_W(1) = cos(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))*-2.713306697987719E-1+cos(phi)*sin(psi)*1.612305434011315E-15-sin(phi)*sin(psi)*1.316547297686338E+1+sin(q4)*(sin(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))*1.0-cos(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)))*6.158789907116443+cos(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)*1.383065284555778E+1-sin(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16))*2.713306697987719E-1-cos(q6)*(sin(q4)*(sin(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))*1.0-cos(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)))-cos(q4)*(cos(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)-sin(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)*1.0))*1.816157378576463-sin(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)*1.383065284555778E+1-sin(q6)*(sin(q5)*(cos(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))+sin(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)))-cos(q5)*(cos(q4)*(sin(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))*1.0-cos(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)))*1.0+sin(q4)*(cos(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)-sin(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)*1.0))*1.0)*1.816157378576463-cos(q4)*(cos(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)-sin(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)*1.0)*6.158789907116443+cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)*7.894146239959809E-2-sin(q5)*(cos(q4)*(sin(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))*1.0-cos(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)))*1.0+sin(q4)*(cos(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)-sin(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)*1.0))*1.317188700014071E-4-cos(q5)*(cos(q3)*(cos(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)+cos(psi)*cos(theta)*sin(q1))+sin(q3)*(cos(q2)*(sin(q1)*(cos(phi)*sin(psi)+sin(phi)*sin(psi)*1.224646799141545E-16+cos(phi)*cos(psi)*sin(theta)*1.224646799141545E-16-cos(psi)*sin(phi)*sin(theta)*1.0)-cos(psi)*cos(q1)*cos(theta)*1.0)+sin(q2)*(cos(phi)*sin(psi)*-1.224646799141545E-16+sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta)+cos(psi)*sin(phi)*sin(theta)*1.224646799141545E-16)))*1.317188700014071E-4-cos(phi)*cos(psi)*sin(theta)*1.316547297686338E+1+cos(psi)*cos(theta)*sin(q1)*7.894146239959809E-2-cos(psi)*sin(phi)*sin(theta)*1.612305434011315E-15; 
		tau_W(2) = 0.0; 
		
		Eigen::Matrix3d R_W2D;
		R_W2D << cos(psi)*cos(theta), cos(theta)*sin(psi), -sin(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(phi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), cos(phi)*cos(theta) ;
		
		Eigen::Vector3d tau_D;
		
		tau_D = R_W2D * tau_W ;
		
		*grav_terms = tau_D ;

	
	
	}
	
	
public: double minn(double x, double y)
	{
		if(x>=y){ 
			return y;
		}else{
			return x;
		}
	}

public: double maxx(double x, double y)
	{
		if(x>=y){
			return x;
		}else{
			return y;
		}
	}


public: void calculateMatrixM(Eigen::MatrixXd* acc_to_sqvel_matrix)
	{
		Eigen::MatrixXd mat(4,8);
		double fcon = 4.63e-04; 
		double mcon = 0.0173;
		double xlen = 0.534;
		double ylen = 0.574;
		mat(0,0) = -ylen*fcon;				mat(0,1) = ylen*fcon; 
		mat(0,2) = ylen*fcon; 				mat(0,3) = -ylen*fcon;
		mat(0,4) = -ylen*fcon;				mat(0,5) = ylen*fcon;
		mat(0,6) = ylen*fcon;				mat(0,7) = -ylen*fcon;

		mat(1,0) = -xlen*fcon;				mat(1,1) = -xlen*fcon;
		mat(1,2) = xlen*fcon;				mat(1,3) = xlen*fcon;
		mat(1,4) = -xlen*fcon;				mat(1,5) = -xlen*fcon;
		mat(1,6) = xlen*fcon;				mat(1,7) = xlen*fcon;

		mat(2,0) = 1*fcon*mcon;	  mat(2,1) = -1*fcon*mcon;		mat(2,2) = 1*fcon*mcon;		mat(2,3) = -1*fcon*mcon;
		mat(2,4) = -1*fcon*mcon;  mat(2,5) = 1*fcon*mcon;		mat(2,6) = -1*fcon*mcon;	mat(2,7) = 1*fcon*mcon;

		mat(3,0) = fcon;			mat(3,1) = fcon;			mat(3,2) = fcon;			mat(3,3) = fcon;
		mat(3,4) = fcon;			mat(3,5) = fcon;			mat(3,6) = fcon;			mat(3,7) = fcon;

		*acc_to_sqvel_matrix = mat.transpose() * (mat * mat.transpose()).inverse() ; 
	    
	}


public: void bumper_callback(const sensor_msgs::Range& msg)
{
	std::ofstream ufile;
  	ufile.open("/home/dc4204/Downloads/thesis_code/testbench/harrier_3DoF/gazebo_results/Visual_Grasping/RANGE_FINDER.txt", std::ios::out | std::ios::app);
    ufile << time << ", " << msg.range << "\n" ;
    ufile.close();	

	if(ticker>5){
		return;
	}
	if(msg.range>0.15){
		return;
	}else{
		//GO INTO GRASPING MODE	
		fingers = 1;
		ticker++;
		if(ticker>4){
			des_y = 0.0;
			des_z = 1.4;
		}	
	}
		
}



public: void aruco_callback( const std_msgs::Float64MultiArray& aruco_msg )
	{
		if(!take_off_complete){
			return;
		}
		if(aruco_counter==-1){
			return;
		}
		vector< Eigen::Matrix3d > R_mats ;
		vector< int > ids ;
		vector< Eigen::Vector3d > t_vecs;
	
		//int cols = aruco_msg.layout.dim[0].size ;  //set to 13: id r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3
		int rows = aruco_msg.layout.dim[1].size ;  //number of markers seen
		
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		
		int count = 0;
		for (int i = 0 ; i < rows ; i++){ // for every marker
			// Get the ID
			int id = (int) aruco_msg.data[count];
			ids.push_back(id); 
			
			// Get the Rotation Matrix
			for (int k = 0 ; k < 3 ; k++){
				for (int l = 0 ; l < 3 ; l++){
					count++;
					R(k,l) = aruco_msg.data[count];
				}
			}
			R_mats.push_back(R);
			
			// Get the Translation Vector
			for (int m = 0 ; m < 3 ; m++){
				count++; 
				t(m) = aruco_msg.data[count];
				
			}
			t_vecs.push_back(t);
			count++; // to set it up for the next id, no problem if there's not another one, since count is just the iter variable, not the break variable
		}

		// Only marker with ID-1 is visible here.
		Eigen::Vector3d Pdw = drone_pos ;
		Eigen::Matrix3d Rdw = drone_rot.toRotationMatrix();
		Eigen::Vector3d Pdc_d ;
		Pdc_d << 0, 0.1, 0.2 ; 
		Eigen::Matrix3d Rcd ;
		Rcd << 1.0, 0.0, 0.0, 0.0, cos(-M_PI/2.0 ), -sin( -M_PI/2.0 ), 0.0, sin(-M_PI/2.0), cos(-M_PI/2.0) ;
		
		Eigen::Matrix3d Rmc = R ;   // from marker to camera
		Eigen::Vector3d Pcm_c = t ; // from camera to marker in camera frame
		Eigen::Vector3d Pmo_m ;
		Pmo_m << 0.6, 0.0, 0.35 ;   
		
		Eigen::Vector3d P_wo = Pdw + Rdw * Pdc_d + Rdw * Rcd * Pcm_c + Rdw * Rcd * Rmc * Pmo_m ;
		aruco_counter++;
		object += P_wo ;
		int num = 30;
		if(aruco_counter==num){
			object << object[0]/num, object[1]/num, object[2]/num ;
			aruco_counter = -1;
			std::cout << object[0] << " , " << object[1] << " , " << object[2] << std::endl;
		}
	}



public: void keyper_callback(const std_msgs::Int16& keyper_msg)
	{
		if(Land){
			return;
		}
		
		if(!landing_gear_retracted){
			return;
		}
	
		int code = keyper_msg.data;
		switch(code)
		{		
			case 1:
				des_yaw = des_yaw + 0.1745;
				//puts("Turning left");
				break;
			case 2:
				des_yaw = des_yaw - 0.1745;				
				//puts("Turning right");
				break;
			case 3:
				des_z = des_z + 0.3;
				//puts("Going up");
				break;
			case 4:
				des_z = des_z - 0.3;
				//puts("Going down");
				break;
			case 5:
				des_x = des_x + 0.3;
				//puts("Going X forward");
				break;
			case 6:
				des_x = des_x - 0.3;
				//puts("Going X backward");
				break;
			case 7:
				des_y = des_y + 0.3;
				//puts("Going Y forward");
				break;
			case 8:
				des_y = des_y - 0.3;
				//puts("Going Y backward");
				break;
			case 9:
				Land = true;
				land_time = time;
				break;
		}
	}
	
	
public: void gear_callback(const sensor_msgs::JointState joint_msg)
	{
		
		if(!gear_bool){
			return;
		}
		
		if(!take_off_complete){
			return;
		}
		

		std::vector< double > angles, rates ;
		std::vector< std::string > names ;

		double des1, des2;
		des1 = -1.55;	des2 = 1.55;
		if(Land){
			des1 = des1 + 0.4*(time-land_time);
			des1 = minn(-0.05,des1);
			des2 = des2 - 0.4*(time-land_time);
			des2 = maxx(0.05,des2);
		}
		
		names = joint_msg.name ;
		angles = joint_msg.position ;
		rates = joint_msg.velocity ;
		double ang1, ang2, vel1, vel2 ;
/*	
		if(names[0]=="harrierD7::harrierD7/land1_joint"){
			ang1 = angles[0];
			ang2 = angles[1];
			vel1 = rates[0];
			vel2 = rates[1];
		}else{
			ang1 = angles[1];
			ang2 = angles[0];
			vel1 = rates[1];
			vel2 = rates[0];
		}
*/
		ang1 = angles[0];
		ang2 = angles[1];
		vel1 = rates[0];
		vel2 = rates[1];

		if(!landing_gear_retracted){
			if( (my_abs(ang1 + 1.263)<1e-2) && (my_abs(ang2 - 1.263)<1e-2) ){
				if( (my_abs(vel1)<1e-1) && (my_abs(vel2)<1e-1) ){
					landing_gear_retracted = 1;
					std::cout << "\nLanding Gear Retracted Successfully... \n   Aerial Manipulation may begin" << std::endl;
					std::cout << "\nAll Teleoperation Services have now been enabled" << std::endl;
				}
			}
		}

		if(Land){
			if( (my_abs(ang1)<2e-1) && (my_abs(ang2)<2e-1) ){
				ready_to_land = true;
			}
		}
			
		double Kp = 3.5 ;
		double Kd = 0.14 ;
	
		double t1 = -Kp*(ang1 - des1) - Kd*vel1 ;
		double t2 = -Kp*(ang2 - des2) - Kd*vel2 ;	
				
		gear1->SetForce(0,t1);
		gear2->SetForce(0,t2);
		
	}

	

public: void kinova_callback(const std_msgs::Int16& keyper_msg)
	{
		if(Land){
			return;
		}
	
		if(!landing_gear_retracted){
			return;
		}
	
		int code = keyper_msg.data;
		switch(code)
		{		
			case 1:
				q_desired(0) = q_desired(0) + 0.1745;
				break;
			case 2:
				q_desired(0) = q_desired(0) - 0.1745;				
				break;
			case 3:
				q_desired(1) = q_desired(1) + 0.1745;
				break;
			case 4:
				q_desired(1) = q_desired(1) - 0.1745;
				break;
			case 5:
				q_desired(2) = q_desired(2) + 0.1745;
				break;
			case 6:
				q_desired(2) = q_desired(2) - 0.1745;
				break;
			case 7:
				q_desired(3) = q_desired(3) + 0.1745;
				break;
			case 8:
				q_desired(3) = q_desired(3) - 0.1745;
				break;
			case 9:
				q_desired(4) = q_desired(4) + 0.1745;
				break;
			case 10:
				q_desired(4) = q_desired(4) - 0.1745;
				break;
			case 11:
				q_desired(5) = q_desired(5) + 0.1745;
				break;
			case 12:
				q_desired(5) = q_desired(5) - 0.1745;
				break;
			case 13:
				q_desired(6) = q_desired(6) + 0.1745;
				break;
			case 14:
				q_desired(6) = q_desired(6) - 0.1745;
				break;
			case 15:
				fingers = 0;
				break;
			case 16:
				fingers = 1;
				break;			
		}
	}
	



};
GZ_REGISTER_MODEL_PLUGIN(UASControlPlugin)
}
