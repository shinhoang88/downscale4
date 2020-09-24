#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>

#include "downscale4/one.h"
#include "downscale4/two.h"
#include "downscale4/three.h"
#include "downscale4/seven.h"



#include "ros/ros.h"
#include "ros/time.h"
#include </home/nuclear/eigen/Eigen/Dense>
#include </home/nuclear/eigen/Eigen/pseudoinverse.cpp>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>


#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#define pi 3.14159265359
#define g_acc 9.81

//Arm offset (m)
#define d1 0.371
#define d3 0.547
#define d5 0.484
#define d7 0.273
//Center of mass for each joint, from local coordinate (m)
#define lg1 0.247
#define lg3 0.365
#define lg5 0.323
#define lg7 0.273

//#define lg1 0
//#define lg3 0
//#define lg5 0
//#define lg7 0

//Joint mass (kg)
#define m1 3.8202
#define m2 2.9008
#define m3 2.0424
#define m4 1.7764
#define m5 1.1622
#define m6 1.12
#define m7 0.15542

//#define m1 4
//#define m2 8.59206
//#define m3 4
//#define m4 4.85898
//#define m5 2
//#define m6 2.87293
//#define m7 0.25167


double SI,SI_COM,SI_HIG,SI_LOW,SI_FIX;
double minT2;
double q[7],T[7];
double joint1v, joint2v, joint3v, joint4v, joint5v, joint6v, joint7v;
using Eigen::MatrixXd;
using namespace std;

//  global parameters
const int	N = 1000;
double	dt;
//const int 	duration = 6;	//seconds



double		initial_pose_end_effector[3];	//x,y,z init postion for each trajectory _ computed by Forward Kinematics
double 		cur_tta[7];		//current joint angles _ from  msgCallbackP function
double 		cur_tau[7];		//current joint torques _ from  msgCallbackT function
double 		cur_vel[7];
bool 		is_trajectory_possible;


ros::Publisher pub_jointp;
ros::Publisher pub_jointt;
ros::Subscriber sub_jointp;
ros::Subscriber sub_jointv;
ros::Subscriber sub_jointt;

//Given
MatrixXd P_INIT(4,4);	//Given init homogeneous transformation matrix
MatrixXd P_END(4,4);	//Given end homogeneous transformation matrix
MatrixXd P_COM(4,4);	//Given end homogeneous transformation matrix
MatrixXd T_07(4,4);		//Forward kinematics
//Temp matrices for calculating
MatrixXd W(3,1); 		//Wrist position relating to base
MatrixXd DZ(3,1);		//[0; 0; 1] matrix
MatrixXd L_SW(3,1); 	//Wrist position relating to shoulder
MatrixXd D1(3,1); 		//D1 column matrix
MatrixXd KSW(3,3); 		//Skew symetric matrix of vector between wrist and shoulder
MatrixXd USW(3,1); 		//Unit vector of L_SW
MatrixXd R03dot(3,3); 		//Rotation matrix of the ref joint angles q1dot, q2dot
MatrixXd XS(3,3); 		//Shoulder constant matrix
MatrixXd YS(3,3); 		//
MatrixXd ZS(3,3); 		//
MatrixXd I(3,3); 		//Identity matrix
MatrixXd R34(3,3); 		//Rotation matrix from joint 3 to joint 4
MatrixXd XW(3,3); 		//Wrist constant matrix
MatrixXd YW(3,3); 		//
MatrixXd ZW(3,3); 		//

MatrixXd Jacob(6,7);
MatrixXd pinv_Jacob(7,6);
MatrixXd Trans_Jacob(7,6);
MatrixXd Tau(7,1);
MatrixXd GC_Tau(7,1);
MatrixXd theta(7,1);
MatrixXd desire_q(7,1);
MatrixXd pos_err(3,1);
MatrixXd pos_desired(3,1);
MatrixXd vel_err(7,1);
MatrixXd RotMat(3,3); 		//EE rotation matrix
MatrixXd RotMat_desired(3,3); 		//EE desired rotation matrix
MatrixXd RotMat_err(3,3); 		//EE rotation matrix error
MatrixXd RotVec_err(3,1); 		//EE rotation vecto error
MatrixXd PosEE(3,1); 		//EE postion
MatrixXd Pos_prev(3,1); 		//EE postion
MatrixXd Pose_err(6,1); 		//EE pose error 6D [pos_err; RotVec_err]
MatrixXd tool_vel(6,1); 		//tool frame velocity

MatrixXd Stiffness(6,6); 	//stiffness diagonal matrix
MatrixXd StifVec(6,1); 	//stiffness diagonal vector

MatrixXd Damping(6,6); 	//damping diagonal matrix
MatrixXd DampVec(6,1); 	//damping diagonal vector

//*********************Call Back***********************************
void msgCallbackP(const downscale4::seven::ConstPtr& msg)
{

	cur_tta[0] = msg->a;
	cur_tta[1] = msg->b;
	cur_tta[2] = msg->c;
	cur_tta[3] = msg->d;
	cur_tta[4] = msg->e;
	cur_tta[5] = msg->f;
	cur_tta[6] = msg->g;

}

void msgCallbackT(const downscale4::seven::ConstPtr& msg)
{

	cur_tau[0] = msg->a;
	cur_tau[1] = msg->b;
	cur_tau[2] = msg->c;
	cur_tau[3] = msg->d;
	cur_tau[4] = msg->e;
	cur_tau[5] = msg->f;
	cur_tau[6] = msg->g;


}
void msgCallbackv(const downscale4::seven::ConstPtr& msg)
{

	cur_vel[0] = msg->a;
	cur_vel[1] = msg->b;
	cur_vel[2] = msg->c;
	cur_vel[3] = msg->d;
	cur_vel[4] = msg->e;
	cur_vel[5] = msg->f;
	cur_vel[6] = msg->g;

}

//**********************************Inverse Kinematics***********************************************************
void InvK7(const Eigen::MatrixXd &data, double si)
{

	double q1dot, q2dot;
	double norm_L_SW;
	int GC2, GC4, GC6;
	//Given homogeneous transformation matrix

	DZ << 	0, 0, 1;
	D1 << 	0, 0, d1;
	I = I.setIdentity(3,3);

	//Compute wrist position relating to base from the given EE position end orientation
	W = data.topRightCorner(3,1)-d7*(data.topLeftCorner(3,3)*DZ);	//P(1:3,4)
	//Compute wrist position relating to shoulder
	L_SW = W - D1;
	norm_L_SW = L_SW.norm();
	//Elbow angle q4 in radian
	q[3] = acos((pow(norm_L_SW,2) - pow(d3,2) - pow(d5,2))/(2*d3*d5));
	if (q[3]>=0)GC4 = 1;
	else GC4 = -1;
	//Compute skew symetric matrix of the vector between wrist and shoulder:
	USW = L_SW/norm_L_SW;
	KSW << 	0, -USW(2), USW(1),
			USW(2), 0, -USW(0),
			-USW(1), USW(0), 0;
	//Set q3=0, compute reference joint angle q1dot, q2dot of the ref plane
	q1dot = atan2(W(1),W(0));
	q2dot = pi/2 - asin((W(2)-d1)/norm_L_SW) - acos((pow(d3,2)+pow(norm_L_SW,2)-pow(d5,2))/(2*d3*norm_L_SW));
	//Rotation matrix of the ref joint angles q1dot, q2dot:
	R03dot << 	cos(q1dot)*cos(q2dot),	-cos(q1dot)*sin(q2dot),	-sin(q1dot),
				cos(q2dot)*sin(q1dot),	-sin(q1dot)*sin(q2dot),	cos(q1dot),
				-sin(q2dot),			-cos(q2dot),			0;
	//Shoulder constant matrices Xs Ys Zs
	XS = KSW*R03dot;
	YS = -(KSW*KSW)*R03dot;
	ZS = (I+KSW*KSW)*R03dot;
	//constant matrices Xw Yw Zw
	R34 << 		cos(q[3]),		0,		sin(q[3]),
				sin(q[3]),		0, 		-cos(q[3]),
				0,				1,		0;
	XW = R34.transpose()*XS.transpose()*data.topLeftCorner(3,3);
	YW = R34.transpose()*YS.transpose()*data.topLeftCorner(3,3);
	ZW = R34.transpose()*ZS.transpose()*data.topLeftCorner(3,3);

	//Theta2
	q[1] = acos(-sin(si)*XS(2,1)-cos(si)*YS(2,1)-ZS(2,1));
	if (q[1]>=0)GC2 = 1;
	else GC2 = -1;
	//Theta1, theta3
	q[0] = atan2(GC2*(-sin(si)*XS(1,1)-cos(si)*YS(1,1)-ZS(1,1)),GC2*(-sin(si)*XS(0,1)-cos(si)*YS(0,1)-ZS(0,1)));
	q[2] = atan2(GC2*(sin(si)*XS(2,2)+cos(si)*YS(2,2)+ZS(2,2)),GC2*(-sin(si)*XS(2,0)-cos(si)*YS(2,0)-ZS(2,0)));

	//Theta6
	q[5] = acos(sin(si)*XW(2,2)+cos(si)*YW(2,2)+ZW(2,2));
	if (q[5]>=0)GC6 = 1;
	else GC6 = -1;
	//Theta5, theta7
	q[4] = atan2(GC6*(sin(si)*XW(1,2)+cos(si)*YW(1,2)+ZW(1,2)),GC6*(sin(si)*XW(0,2)+cos(si)*YW(0,2)+ZW(0,2)));
	q[6] = atan2(GC6*(sin(si)*XW(2,1)+cos(si)*YW(2,1)+ZW(2,1)),GC6*(-sin(si)*XW(2,0)-cos(si)*YW(2,0)-ZW(2,0)));

}

//**********************************Forward Kinematics***********************************************************
void T2_Opt(double SI_L, double SI_H)
{
	clock_t begin = clock();
	double G2y,min_T2;
	min_T2 = 50;

	double i;
	for(i=SI_L; i<SI_H; )
	{
		InvK7(P_COM, i);
		//lg3=2d3/2 (m3+m2)
		G2y = fabs(g_acc*(m7*(lg7*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) + m6*(d5*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) + m5*(lg5*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) + d3*m4*cos(q[1])*sin(q[0]) + lg3*m3*cos(q[1])*sin(q[0])));
		//G2y = fabs(g_acc*(m7*(d7*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) + m6*((d5+d7/3)*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) + m5*((5*d5/6)*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) + (d3+d5/3)*m4*cos(q[1])*sin(q[0]) + ((m3*5*d3/6)+(m2*d3/3))*cos(q[1])*sin(q[0])));
		//G2y = fabs(-g_acc*(m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + m6*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + d3*m4*sin(q[1]) + lg3*m3*sin(q[1])));
		//G2y = fabs(g_acc*(m6*(d5*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[0])*cos(q[1])) + m5*(lg5*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[0])*cos(q[1])) + m7*(lg7*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[0])*cos(q[1])) + d3*m4*cos(q[0])*cos(q[1]) + lg3*m3*cos(q[0])*cos(q[1])));
		if (min_T2>G2y)
		{
			min_T2=G2y;
			SI_COM = i;
		}
		G2y = 0;
		i = i+0.005;
	}
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

	//ROS_INFO("Optimal [Si , Torque, Elapse time] : [%f , %f, %f]",SI_COM,min_T2,elapsed_secs);
	minT2 = min_T2;
}
//**********************************Forward Kinematics***********************************************************
void ForwK7(Eigen::MatrixXd &rot, Eigen::MatrixXd &pos, double tta[7])
{
	//input q[7] -> &data[7]
	rot(0,0) = cos(tta[6])*(sin(tta[5])*(sin(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) - cos(tta[0])*cos(tta[3])*sin(tta[1])) - cos(tta[5])*(cos(tta[4])*(cos(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) + cos(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[2])*sin(tta[0]) + cos(tta[0])*cos(tta[1])*sin(tta[2])))) + sin(tta[6])*(sin(tta[4])*(cos(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) + cos(tta[0])*sin(tta[1])*sin(tta[3])) - cos(tta[4])*(cos(tta[2])*sin(tta[0]) + cos(tta[0])*cos(tta[1])*sin(tta[2])));
	rot(0,1) = cos(tta[6])*(sin(tta[4])*(cos(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) + cos(tta[0])*sin(tta[1])*sin(tta[3])) - cos(tta[4])*(cos(tta[2])*sin(tta[0]) + cos(tta[0])*cos(tta[1])*sin(tta[2]))) - sin(tta[6])*(sin(tta[5])*(sin(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) - cos(tta[0])*cos(tta[3])*sin(tta[1])) - cos(tta[5])*(cos(tta[4])*(cos(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) + cos(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[2])*sin(tta[0]) + cos(tta[0])*cos(tta[1])*sin(tta[2]))));
	rot(0,2) = - cos(tta[5])*(sin(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) - cos(tta[0])*cos(tta[3])*sin(tta[1])) - sin(tta[5])*(cos(tta[4])*(cos(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) + cos(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[2])*sin(tta[0]) + cos(tta[0])*cos(tta[1])*sin(tta[2])));

	rot(1,0) = - cos(tta[6])*(sin(tta[5])*(sin(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) + cos(tta[3])*sin(tta[0])*sin(tta[1])) - cos(tta[5])*(cos(tta[4])*(cos(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) - sin(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[0])*cos(tta[2]) - cos(tta[1])*sin(tta[0])*sin(tta[2])))) - sin(tta[6])*(sin(tta[4])*(cos(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) - sin(tta[0])*sin(tta[1])*sin(tta[3])) - cos(tta[4])*(cos(tta[0])*cos(tta[2]) - cos(tta[1])*sin(tta[0])*sin(tta[2])));
	rot(1,1) = sin(tta[6])*(sin(tta[5])*(sin(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) + cos(tta[3])*sin(tta[0])*sin(tta[1])) - cos(tta[5])*(cos(tta[4])*(cos(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) - sin(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[0])*cos(tta[2]) - cos(tta[1])*sin(tta[0])*sin(tta[2])))) - cos(tta[6])*(sin(tta[4])*(cos(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) - sin(tta[0])*sin(tta[1])*sin(tta[3])) - cos(tta[4])*(cos(tta[0])*cos(tta[2]) - cos(tta[1])*sin(tta[0])*sin(tta[2])));
	rot(1,2) = cos(tta[5])*(sin(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) + cos(tta[3])*sin(tta[0])*sin(tta[1])) + sin(tta[5])*(cos(tta[4])*(cos(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) - sin(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[0])*cos(tta[2]) - cos(tta[1])*sin(tta[0])*sin(tta[2])));

	rot(2,0) = sin(tta[6])*(sin(tta[4])*(cos(tta[1])*sin(tta[3]) + cos(tta[2])*cos(tta[3])*sin(tta[1])) + cos(tta[4])*sin(tta[1])*sin(tta[2])) - cos(tta[6])*(cos(tta[5])*(cos(tta[4])*(cos(tta[1])*sin(tta[3]) + cos(tta[2])*cos(tta[3])*sin(tta[1])) - sin(tta[1])*sin(tta[2])*sin(tta[4])) + sin(tta[5])*(cos(tta[1])*cos(tta[3]) - cos(tta[2])*sin(tta[1])*sin(tta[3])));
	rot(2,1) = cos(tta[6])*(sin(tta[4])*(cos(tta[1])*sin(tta[3]) + cos(tta[2])*cos(tta[3])*sin(tta[1])) + cos(tta[4])*sin(tta[1])*sin(tta[2])) + sin(tta[6])*(cos(tta[5])*(cos(tta[4])*(cos(tta[1])*sin(tta[3]) + cos(tta[2])*cos(tta[3])*sin(tta[1])) - sin(tta[1])*sin(tta[2])*sin(tta[4])) + sin(tta[5])*(cos(tta[1])*cos(tta[3]) - cos(tta[2])*sin(tta[1])*sin(tta[3])));
	rot(2,2) = cos(tta[5])*(cos(tta[1])*cos(tta[3]) - cos(tta[2])*sin(tta[1])*sin(tta[3])) - sin(tta[5])*(cos(tta[4])*(cos(tta[1])*sin(tta[3]) + cos(tta[2])*cos(tta[3])*sin(tta[1])) - sin(tta[1])*sin(tta[2])*sin(tta[4]));

	pos(0,0) = d3*cos(tta[0])*sin(tta[1]) - d7*(cos(tta[5])*(sin(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) - cos(tta[0])*cos(tta[3])*sin(tta[1])) + sin(tta[5])*(cos(tta[4])*(cos(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) + cos(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[2])*sin(tta[0]) + cos(tta[0])*cos(tta[1])*sin(tta[2])))) - d5*(sin(tta[3])*(sin(tta[0])*sin(tta[2]) - cos(tta[0])*cos(tta[1])*cos(tta[2])) - cos(tta[0])*cos(tta[3])*sin(tta[1]));
	pos(1,0) = d5*(sin(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) + cos(tta[3])*sin(tta[0])*sin(tta[1])) + d7*(cos(tta[5])*(sin(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) + cos(tta[3])*sin(tta[0])*sin(tta[1])) + sin(tta[5])*(cos(tta[4])*(cos(tta[3])*(cos(tta[0])*sin(tta[2]) + cos(tta[1])*cos(tta[2])*sin(tta[0])) - sin(tta[0])*sin(tta[1])*sin(tta[3])) + sin(tta[4])*(cos(tta[0])*cos(tta[2]) - cos(tta[1])*sin(tta[0])*sin(tta[2])))) + d3*sin(tta[0])*sin(tta[1]);
	pos(2,0) = d1 - d7*(sin(tta[5])*(cos(tta[4])*(cos(tta[1])*sin(tta[3]) + cos(tta[2])*cos(tta[3])*sin(tta[1])) - sin(tta[1])*sin(tta[2])*sin(tta[4])) - cos(tta[5])*(cos(tta[1])*cos(tta[3]) - cos(tta[2])*sin(tta[1])*sin(tta[3]))) + d5*(cos(tta[1])*cos(tta[3]) - cos(tta[2])*sin(tta[1])*sin(tta[3])) + d3*cos(tta[1]);

//	pos[0] = d3*cos(q(1))*sin(q(2)) - d7*(cos(q(6))*(sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) - cos(q(1))*cos(q(4))*sin(q(2))) + sin(q(6))*(cos(q(5))*(cos(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) + cos(q(1))*sin(q(2))*sin(q(4))) + sin(q(5))*(cos(q(3))*sin(q(1)) + cos(q(1))*cos(q(2))*sin(q(3))))) - d5*(sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) - cos(q(1))*cos(q(4))*sin(q(2)));
//
//	pos[1] = d5*(sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) + cos(q(4))*sin(q(1))*sin(q(2))) + d7*(cos(q(6))*(sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) + cos(q(4))*sin(q(1))*sin(q(2))) + sin(q(6))*(cos(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) - sin(q(1))*sin(q(2))*sin(q(4))) + sin(q(5))*(cos(q(1))*cos(q(3)) - cos(q(2))*sin(q(1))*sin(q(3))))) + d3*sin(q(1))*sin(q(2));
//
//	pos[2] = d1 - d7*(sin(q(6))*(cos(q(5))*(cos(q(2))*sin(q(4)) + cos(q(3))*cos(q(4))*sin(q(2))) - sin(q(2))*sin(q(3))*sin(q(5))) - cos(q(6))*(cos(q(2))*cos(q(4)) - cos(q(3))*sin(q(2))*sin(q(4)))) + d5*(cos(q(2))*cos(q(4)) - cos(q(3))*sin(q(2))*sin(q(4))) + d3*cos(q(2));


}


//*************************Convert rotation matrix to rotation vector*********************************************
void rotMat2rotVec(Eigen::MatrixXd &rotvec, Eigen::MatrixXd &rotmat)
{
	double th;
	th =  acos(0.5*(rotmat(0,0)+rotmat(1,1)+rotmat(2,2)-1));
	if (fabs(th)<0.01)
	{
		rotvec(0,0) = 0;
		rotvec(1,0) = 0;
		rotvec(2,0) = 0;
	}
	else
	{
		rotvec(0,0) = (th*(rotmat(2,1)-rotmat(1,2)))/(2*sin(th));
		rotvec(1,0) = (th*(rotmat(0,2)-rotmat(2,0)))/(2*sin(th));
		rotvec(2,0) = (th*(rotmat(1,0)-rotmat(0,1)))/(2*sin(th));
	}
}
//*******************************Compute Cartesian Trajectory*****************************************************
void calc_cartezian_trajectory(double CT[3][N], double ini[3], double des[3])
{

	double trajectory_length;		//the length from P0 -> P1
	double trajectory_dir_vec[3];	//trajectory direction vector
	double v_max;					//maximum velocity
	double t=0; 					//incremental running time
	double step_size[N];			//counting step
	int i=1;


	double dt, duration;

	dt	= 0.01;						//control command period
	duration = dt * N;				//total time

	trajectory_length = sqrt(pow((des[0] - ini[0]),2) + pow((des[1] - ini[1]),2) + pow((des[2] - ini[2]),2));

	//ROS_INFO("trajectory length: %f", trajectory_length);

	trajectory_dir_vec[0] = (des[0] - ini[0]) / trajectory_length;
	trajectory_dir_vec[1] = (des[1] - ini[1]) / trajectory_length;
	trajectory_dir_vec[2] = (des[2] - ini[2]) / trajectory_length;

	//ROS_INFO("trajectory x direction: %f",trajectory_dir_vec[0]);
	//ROS_INFO("trajectory y direction: %f",trajectory_dir_vec[1]);
	//ROS_INFO("trajectory z direction: %f",trajectory_dir_vec[2]);

	v_max = (4 * trajectory_length)/(3 * duration);

	//ROS_INFO("max velocity: %f",v_max);

	//the trajectory divided to three different parts.
	//1st part is quarter of the duration time that have acceleration.
	//2nd part is two quarter of the duration time with steady velocity.
	//3rd part is another quarter of the duration time that have deceleration.
	step_size[0] = (2 * v_max / duration) * pow(dt,2);		//acceleration = 4*v_max/duration (1/2 at^2)
	t+=dt;
	while (t < duration / 4) //1st part
	{
		step_size[i] = (4 * v_max /duration) * t * dt + (2 * v_max /duration) * pow(dt,2);
		i++;
		t+=dt;
	}
	while (t < 3 * duration / 4) //2nd part
	{
		step_size[i] = v_max * dt;
		i++;
		t+=dt;
	}
	while (t <= duration) //3rd part
	{
		step_size[i] = (4 * v_max - 4 / duration * v_max * t) * dt - 2 * v_max / duration * pow(dt,2);
		i++;
		t+=dt;
	}

	//calculating the cartesian trajectory
	for (i=0; i<3; i++)
	{
		CT[i][0] = step_size[0] * trajectory_dir_vec[i] + ini[i];
	}
	ROS_INFO("cartesian step #1 : [%f , %f , %f]",CT[0][0],CT[1][0],CT[2][0]);
	ROS_INFO("");
	for (i=1; i<N ; i++)
	{
		for (int j=0; j<3 ; j++)
		{
			CT[j][i] = step_size[i] * trajectory_dir_vec[j] + CT[j][i-1];
		}
		ROS_INFO("cartezian step #%d : [%f , %f , %f]",i+1,CT[0][i],CT[1][i],CT[2][i]);
	}

}

//bool move_arm(downscale4::MoveArm::Request  &req, downscale4::MoveArm::Response &res)
//{
//
//
//
//	return true;
//}

//****************************Pos Publishing message*********************
void jointp_publish(Eigen::MatrixXd &tta)
{
	downscale4::seven msgp;
	msgp.a = tta(0,0);
	msgp.b = tta(1,0);
	msgp.c = tta(2,0);
	msgp.d = tta(3,0);
	msgp.e = tta(4,0);
	msgp.f = tta(5,0);
	msgp.g = tta(6,0);

	pub_jointp.publish(msgp);

}

//****************************Tau Publishing message*********************
void jointt_publish(Eigen::MatrixXd &tau)
{
	downscale4::seven msgp;
	msgp.a = tau(0,0);
	msgp.b = tau(1,0);
	msgp.c = tau(2,0);
	msgp.d = tau(3,0);
	msgp.e = tau(4,0);
	msgp.f = tau(5,0);
	msgp.g = tau(6,0);

	pub_jointt.publish(msgp);

}

using namespace KDL;
//*********************main****************************************
int main(int argc, char **argv)
{
        std::string sep = "\n----------------------------------------\n"; //print
	ros::init(argc, argv, "torque_downscale");  // Node name initialization
	ros::NodeHandle nh;                            // Node handle declaration for communication with ROS

	//ros::ServiceServer service = nh.advertiseService("move_arm", move_arm);
	pub_jointp = nh.advertise<downscale4::seven>("gazebo/downscale_jointp", 100);
	pub_jointt = nh.advertise<downscale4::seven>("gazebo/downscale_jointt", 100);

	sub_jointp = nh.subscribe("downscale_actp", 100, msgCallbackP);
	sub_jointt = nh.subscribe("downscale_actt", 100, msgCallbackT);
	sub_jointv = nh.subscribe("downscale_actv", 100, msgCallbackv);



	//Definition of a kinematic chain & add segments to the chain
	KDL::Chain RB7DOFKdl;

//	//Rotx(-90) then translate along new y -d1
//	Frame frame1 = Frame(Rotation::EulerZYX(0.0, 0.0, -pi / 2)) * Frame(Vector(0.0, -d1, 0.0));
//	//Rotx(90)
//	Frame frame2 = Frame(Rotation::EulerZYX(0.0, 0.0, pi / 2));
//	//Rotx(-90) then translate along new y -d1
//	Frame frame3 = Frame(Rotation::EulerZYX(0.0, 0.0, -pi / 2)) * Frame(Vector(0.0, -d3, 0.0));
//	//Rotx(90)
//	Frame frame4 = Frame(Rotation::EulerZYX(0.0, 0.0, pi / 2));
//	//Rotx(-90) then translate along new y -d1
//	Frame frame5 = Frame(Rotation::EulerZYX(0.0, 0.0, -pi / 2)) * Frame(Vector(0.0, -d5, 0.0));
//	//Rotx(90)
//	Frame frame6 = Frame(Rotation::EulerZYX(0.0, 0.0, pi / 2));
//	//Translate along z d7
//	Frame frame7 = Frame(Vector(0.0, 0.0, d7));
//
//
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotY),frame1, KDL::RigidBodyInertia(m1, KDL::Vector(0.0,0,d1), KDL::RotationalInertia(0.031219, 0.011707, 0.019626, 0, 0, 0))));			//Joint1
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotY),frame2, KDL::RigidBodyInertia(m2, KDL::Vector(0.0,-d3/2,0), KDL::RotationalInertia(0.057775, 0.045169, 0.014907, 0, 0, 0))));			//Joint2
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotY),frame3, KDL::RigidBodyInertia(m3, KDL::Vector(0.0,0,d3), KDL::RotationalInertia(0.013794, 0.004203, 0.0096429, 0, 0, 0))));		//Joint3
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotY),frame4, KDL::RigidBodyInertia(m4, KDL::Vector(0.0,-d5/2,0), KDL::RotationalInertia(0.025887, 0.020087, 0.0066873, 0, 0, 0))));			//Joint4
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotY),frame5, KDL::RigidBodyInertia(m5, KDL::Vector(0.0,0,d5), KDL::RotationalInertia(0.0047369, 0.0014734, 0.0032773, 0, 0, 0))));	//Joint5
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotY),frame6, KDL::RigidBodyInertia(m6, KDL::Vector(0.0,-d7/2,0), KDL::RotationalInertia(0.0073673, 0.005407, 0.0025048, 0, 0, 0))));		//Joint6
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotZ),frame7, KDL::RigidBodyInertia(m7, KDL::Vector(0.0,0,d7), KDL::RotationalInertia(0, 0, 0, 0, 0, 0))));												//Joint7

	//translate along z0 d1
        // define configuration of manipua=lator
        // read KDL manual ocoros??
	Joint joint1(Joint::RotZ);
	Frame frame1 = Frame(Vector(0.0, 0.0, d1));
	RigidBodyInertia inert1 = KDL::RigidBodyInertia(m1, KDL::Vector(0.0,0,-d1/5), KDL::RotationalInertia(0.031219, 0.011707, 0.019626, 0, 0, 0));
	//z2=z1
	Joint joint2(Joint::RotY);
	Frame frame2 = Frame(Vector(0.0, 0.0, 0.0));
	//Frame frame2 = Frame(Rotation::EulerZYX(0.0, 0.0, -pi / 2));
	RigidBodyInertia inert2 = KDL::RigidBodyInertia(m2, KDL::Vector(0.0,0,d3/2), KDL::RotationalInertia(0.057775, 0.045169, 0.014907, 0, 0, 0));
	//translate along z2 d3
	Joint joint3(Joint::RotZ);
	Frame frame3 = Frame(Vector(0.0, 0.0, d3));
	//Frame frame3 = Frame(Rotation::EulerZYX(0.0, 0.0, pi / 2)) * Frame(Vector(0.0, 0.0, d3));
	RigidBodyInertia inert3 = KDL::RigidBodyInertia(m3, KDL::Vector(0.0,0,-d3/5), KDL::RotationalInertia(0.013794, 0.004203, 0.0096429, 0, 0, 0));
	//z3=z4
	Joint joint4(Joint::RotY);
	Frame frame4 = Frame(Vector(0.0, 0.0, 0));
	//Frame frame4 = Frame(Rotation::EulerZYX(0.0, 0.0, -pi / 2));
	RigidBodyInertia inert4 = KDL::RigidBodyInertia(m4, KDL::Vector(0.0,0,d5/2), KDL::RotationalInertia(0.025887, 0.020087, 0.0066873, 0, 0, 0));
	//translate along z4 d5
	Joint joint5(Joint::RotZ);
	Frame frame5 = Frame(Vector(0.0, 0.0, d5));
	//Frame frame5 = Frame(Rotation::EulerZYX(0.0, 0.0, pi / 2)) * Frame(Vector(0.0, 0.0, d5));
	RigidBodyInertia inert5 = KDL::RigidBodyInertia(m5, KDL::Vector(0.0,0,-d5/5), KDL::RotationalInertia(0.0047369, 0.0014734, 0.0032773, 0, 0, 0));
	//z5=z6
	Joint joint6(Joint::RotY);
	Frame frame6 = Frame(Vector(0.0, 0.0, 0));
	//Frame frame6 = Frame(Rotation::EulerZYX(0.0, 0.0, -pi / 2));
	RigidBodyInertia inert6 = KDL::RigidBodyInertia(m6, KDL::Vector(0.0,0,d7/2), KDL::RotationalInertia(0.0073673, 0.005407, 0.0025048, 0, 0, 0));
	//translate along z6 d7
	Joint joint7(Joint::RotZ);
	Frame frame7 = Frame(Vector(0.0, 0.0, d7));
	//Frame frame7 = Frame(Rotation::EulerZYX(0.0, 0.0, pi / 2)) * Frame(Vector(0.0, 0.0, 0));
	RigidBodyInertia inert7 = KDL::RigidBodyInertia(m7, KDL::Vector(0.0,0,0), KDL::RotationalInertia(0, 0, 0, 0, 0, 0));

	RB7DOFKdl.addSegment(Segment(joint1,frame1, inert1));			//Joint1
	RB7DOFKdl.addSegment(Segment(joint2,frame2, inert2));			//Joint2
	RB7DOFKdl.addSegment(Segment(joint3,frame3, inert3));		//Joint3
	RB7DOFKdl.addSegment(Segment(joint4,frame4, inert4));			//Joint4
	RB7DOFKdl.addSegment(Segment(joint5,frame5, inert5));	//Joint5
	RB7DOFKdl.addSegment(Segment(joint6,frame6, inert6));		//Joint6
	RB7DOFKdl.addSegment(Segment(joint7,frame7, inert7));

	//Given init homogeneous transformation matrix
	P_INIT << 	-1, 	0, 	0, 		0,
				0, 		1, 	0, 		0.5,
				0, 		0, 	-1, 	0.3,
				0, 		0, 	0, 		1;

	//Given end homogeneous transformation matrix
	P_END << 	1, 	0.9211, 	0.3894, 	0.0,
				0, 	-0.9211, 	-0.3894, 	0.4,
				1, 	0.3894, 	-0.9211, 	0.3,
				0, 	0, 			0, 			1;

	theta = desire_q;

	const int K_stiff = 300;
	const int B_damp = 2;
	int it = 0;
	int max_it = 250;
	double alpha = 0.1;

	//Given position
	pos_desired << 	0.0,
					0.5,
					0.3;
	//Given rotation matrix
	RotMat_desired << 	-1, 	0, 	 0,
						 0, 	1, 	 0,
						 0, 	0, 	-1;


	StifVec << K_stiff, K_stiff, K_stiff, 100, 100, 100;
	Stiffness = StifVec.asDiagonal();

	DampVec << B_damp, B_damp, B_damp, 2, 2, 2;
	Damping = DampVec.asDiagonal();

	SI_FIX = -0.7;
	//Start configuration
	InvK7(P_INIT, SI_FIX);
	theta(0,0) = q[0];
	theta(1,0) = q[1];
	theta(2,0) = q[2];
	theta(3,0) = q[3];
	theta(4,0) = q[4];
	theta(5,0) = q[5];
	theta(6,0) = q[6];



	jointp_publish(theta);
	ros::Duration(0.01).sleep();
	jointp_publish(theta);

	while (ros::ok())
	{

		Pos_prev(0,0) = 0;
		Pos_prev(1,0) = 0;
		Pos_prev(2,0) = 0;

	while ((it==0) || ((pos_err.norm()>0.001)&&(it<max_it)))
	{
		ForwK7(RotMat, PosEE, cur_tta);
		//Position error x,y,z
		pos_err = pos_desired - PosEE;

		//Rotation error
		RotMat_err = RotMat_desired*(RotMat.transpose());
		rotMat2rotVec(RotVec_err, RotMat_err);

		//Pose error

		Pose_err << pos_err, RotVec_err;
//		std::cout << RotMat << sep;
//		std::cout << pos_desired << sep;
//		std::cout << PosEE << sep;
//		std::cout << pos_err << sep;
//		std::cout << Pose_err << sep;
//		std::cout << Pose_err.norm() << sep;

		printf("x: %f y: %f z: %f \n",PosEE(0,0),PosEE(1,0),PosEE(2,0));


		RotVec_err << 0, 0, 0;
		Pose_err << pos_err, RotVec_err;
		// Get some joint pos, vel, acc values
		KDL::JntArray jnt_q(7);				//joint angle
		KDL::JntArray jnt_qd(7);				//joint vel
		KDL::JntArray jnt_qdd(7);			//joint acc
		KDL::JntArray jnt_taugc(7);			//gravity compensation torque
		KDL::Wrenches jnt_wrenches;

		for (unsigned int i = 0; i < 7; i++)
		{
		  jnt_q(i) = cur_tta[i];
		  jnt_qd(i) = 0.0;
		  jnt_qdd(i) = 0.0;
		  jnt_wrenches.push_back(KDL::Wrench());
		}



		//Results Jacobian 6x7
		Jacob(0,0) = - d5*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1])) - d7*(cos(cur_tta[5])*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1])) + sin(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + sin(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])))) - d3*sin(cur_tta[0])*sin(cur_tta[1]);
		Jacob(0,1) = d7*(cos(cur_tta[5])*(cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[0])*cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3])) - sin(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[0])*cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) - cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[4]))) + d5*(cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[0])*cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3])) + d3*cos(cur_tta[0])*cos(cur_tta[1]);
		Jacob(0,2) = d7*(sin(cur_tta[5])*(sin(cur_tta[4])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[3])*cos(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]))) - cos(cur_tta[5])*sin(cur_tta[3])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]))) - d5*sin(cur_tta[3])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]));
		Jacob(0,3) = - d7*(cos(cur_tta[5])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*sin(cur_tta[5])*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1]))) - d5*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3]));
		Jacob(0,4) = d7*sin(cur_tta[5])*(sin(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2])));
		Jacob(0,5) = d7*(sin(cur_tta[5])*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1])) - cos(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + sin(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]))));
		Jacob(0,6) = 0;

		Jacob(1,0) = d3*cos(cur_tta[0])*sin(cur_tta[1]) - d7*(cos(cur_tta[5])*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1])) + sin(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + sin(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2])))) - d5*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1]));
		Jacob(1,1) = d7*(cos(cur_tta[5])*(cos(cur_tta[1])*cos(cur_tta[3])*sin(cur_tta[0]) - cos(cur_tta[2])*sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - sin(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[4]))) + d5*(cos(cur_tta[1])*cos(cur_tta[3])*sin(cur_tta[0]) - cos(cur_tta[2])*sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + d3*cos(cur_tta[1])*sin(cur_tta[0]);
		Jacob(1,2) = d5*sin(cur_tta[3])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])) - d7*(sin(cur_tta[5])*(sin(cur_tta[4])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - cos(cur_tta[3])*cos(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2]))) - cos(cur_tta[5])*sin(cur_tta[3])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])));
		Jacob(1,3) = d7*(cos(cur_tta[5])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*sin(cur_tta[5])*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1]))) + d5*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3]));
		Jacob(1,4) = -d7*sin(cur_tta[5])*(sin(cur_tta[4])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])));
		Jacob(1,5) = -d7*(sin(cur_tta[5])*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1])) - cos(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + sin(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2]))));
		Jacob(1,6) = 0;

		Jacob(2,0) = 0;
		Jacob(2,1) = d7*(sin(cur_tta[5])*(cos(cur_tta[4])*(sin(cur_tta[1])*sin(cur_tta[3]) - cos(cur_tta[1])*cos(cur_tta[2])*cos(cur_tta[3])) + cos(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[4])) - cos(cur_tta[5])*(cos(cur_tta[3])*sin(cur_tta[1]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[3]))) - d3*sin(cur_tta[1]) - d5*(cos(cur_tta[3])*sin(cur_tta[1]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[3]));
		Jacob(2,2) = d7*(sin(cur_tta[5])*(cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[4]) + cos(cur_tta[3])*cos(cur_tta[4])*sin(cur_tta[1])*sin(cur_tta[2])) + cos(cur_tta[5])*sin(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[3])) + d5*sin(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[3]);
		Jacob(2,3) = - d5*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) - d7*(cos(cur_tta[5])*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) + cos(cur_tta[4])*sin(cur_tta[5])*(cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3])));
		Jacob(2,4) = d7*sin(cur_tta[5])*(sin(cur_tta[4])*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) + cos(cur_tta[4])*sin(cur_tta[1])*sin(cur_tta[2]));
		Jacob(2,5) = -d7*(cos(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) - sin(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[4])) + sin(cur_tta[5])*(cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3])));
		Jacob(2,6) = 0;

		Jacob(3,0) = 0;
		Jacob(3,1) = 0;
		Jacob(3,2) = -sin(cur_tta[0]);
		Jacob(3,3) = cos(cur_tta[0])*sin(cur_tta[1]);
		Jacob(3,4) = - cos(cur_tta[2])*sin(cur_tta[0]) - cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]);
		Jacob(3,5) = cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1]) - sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2]));
		Jacob(3,6) = sin(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]));


		Jacob(4,0) = 0;
		Jacob(4,1) = 0;
		Jacob(4,2) = cos(cur_tta[0]);
		Jacob(4,3) = sin(cur_tta[0])*sin(cur_tta[1]);
		Jacob(4,4) = cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2]);
		Jacob(4,5) = sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1]);
		Jacob(4,6) = cos(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])) - sin(cur_tta[4])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3]));

		Jacob(5,0) = 1;
		Jacob(5,1) = 1;
		Jacob(5,2) = 0;
		Jacob(5,3) = cos(cur_tta[1]);
		Jacob(5,4) = sin(cur_tta[1])*sin(cur_tta[2]);
		Jacob(5,5) = cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3]);
		Jacob(5,6) = sin(cur_tta[4])*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) + cos(cur_tta[4])*sin(cur_tta[1])*sin(cur_tta[2]);

//		Jacob(0,0) = - d5*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1])) - d7*(cos(cur_tta[5])*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1])) + sin(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + sin(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])))) - d3*sin(cur_tta[0])*sin(cur_tta[1]);
//		Jacob(0,1) = - d7*(sin(cur_tta[5])*(cos(cur_tta[2])*sin(cur_tta[4]) + cos(cur_tta[3])*cos(cur_tta[4])*sin(cur_tta[2])) + cos(cur_tta[5])*sin(cur_tta[2])*sin(cur_tta[3])) - d5*sin(cur_tta[2])*sin(cur_tta[3]);
//		Jacob(0,2) = cos(cur_tta[0])*(d7*(sin(cur_tta[5])*(cos(cur_tta[2])*sin(cur_tta[4]) + cos(cur_tta[3])*cos(cur_tta[4])*sin(cur_tta[2])) + cos(cur_tta[5])*sin(cur_tta[2])*sin(cur_tta[3])) + d5*sin(cur_tta[2])*sin(cur_tta[3]));
//		Jacob(0,3) = sin(cur_tta[0])*sin(cur_tta[1])*(d7*(cos(cur_tta[3])*cos(cur_tta[5]) - cos(cur_tta[4])*sin(cur_tta[3])*sin(cur_tta[5])) + d5*cos(cur_tta[3])) - d7*cos(cur_tta[1])*sin(cur_tta[4])*sin(cur_tta[5]);
//		Jacob(0,4) = sin(cur_tta[1])*sin(cur_tta[2])*(d5 + d7*cos(cur_tta[5])) + d7*sin(cur_tta[4])*sin(cur_tta[5])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2]));
//		Jacob(0,5) = d7*cos(cur_tta[5])*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1]));
//		Jacob(0,6) = d7*(sin(cur_tta[4])*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) + cos(cur_tta[4])*sin(cur_tta[1])*sin(cur_tta[2]));
//
//		Jacob(1,0) = d3*cos(cur_tta[0])*sin(cur_tta[1]) - d7*(cos(cur_tta[5])*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1])) + sin(cur_tta[5])*(cos(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) + sin(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2])))) - d5*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1]));
//		Jacob(1,1) = d5*(cos(cur_tta[3])*sin(cur_tta[1]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[3])) + d3*sin(cur_tta[1]) - d7*(sin(cur_tta[5])*(cos(cur_tta[4])*(sin(cur_tta[1])*sin(cur_tta[3]) - cos(cur_tta[1])*cos(cur_tta[2])*cos(cur_tta[3])) + cos(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[4])) - cos(cur_tta[5])*(cos(cur_tta[3])*sin(cur_tta[1]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[3])));
//		Jacob(1,2) = sin(cur_tta[0])*(d7*(sin(cur_tta[5])*(cos(cur_tta[2])*sin(cur_tta[4]) + cos(cur_tta[3])*cos(cur_tta[4])*sin(cur_tta[2])) + cos(cur_tta[5])*sin(cur_tta[2])*sin(cur_tta[3])) + d5*sin(cur_tta[2])*sin(cur_tta[3]));
//		Jacob(1,3) = cos(cur_tta[1])*(d7*(cos(cur_tta[5])*sin(cur_tta[3]) + cos(cur_tta[3])*cos(cur_tta[4])*sin(cur_tta[5])) + d5*sin(cur_tta[3])) - cos(cur_tta[0])*sin(cur_tta[1])*(d7*(cos(cur_tta[3])*cos(cur_tta[5]) - cos(cur_tta[4])*sin(cur_tta[3])*sin(cur_tta[5])) + d5*cos(cur_tta[3]));
//		Jacob(1,4) = d7*sin(cur_tta[4])*sin(cur_tta[5])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2])) + d7*cos(cur_tta[4])*sin(cur_tta[1])*sin(cur_tta[2])*sin(cur_tta[5]);
//		Jacob(1,5) = d7*sin(cur_tta[5])*(cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3])) + d7*cos(cur_tta[5])*(sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) - cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1]));
//		Jacob(1,6) = 0;
//
//
//		Jacob(2,0) = 0;
//		Jacob(2,1) = 0;
//		Jacob(2,2) = sin(cur_tta[0])*(d3 + d7*(cos(cur_tta[3])*cos(cur_tta[5]) - cos(cur_tta[4])*sin(cur_tta[3])*sin(cur_tta[5])) + d5*cos(cur_tta[3])) + cos(cur_tta[0])*(d7*(sin(cur_tta[5])*(sin(cur_tta[2])*sin(cur_tta[4]) - cos(cur_tta[2])*cos(cur_tta[3])*cos(cur_tta[4])) - cos(cur_tta[2])*cos(cur_tta[5])*sin(cur_tta[3])) - d5*cos(cur_tta[2])*sin(cur_tta[3]));
//		Jacob(2,3) = d7*cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[4])*sin(cur_tta[5]) - sin(cur_tta[0])*sin(cur_tta[1])*(d7*(cos(cur_tta[5])*sin(cur_tta[3]) + cos(cur_tta[3])*cos(cur_tta[4])*sin(cur_tta[5])) + d5*sin(cur_tta[3]));
//		Jacob(2,4) = (d5 + d7*cos(cur_tta[5]))*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2])) - d7*cos(cur_tta[4])*sin(cur_tta[5])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2]));
//		Jacob(2,5) = -d7*sin(cur_tta[5])*(sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1]));
//		Jacob(2,6) = -d7*(sin(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2])));
//
//		Jacob(3,0) = 0;
//		Jacob(3,1) = 0;
//		Jacob(3,2) = -sin(cur_tta[0]);
//		Jacob(3,3) = cos(cur_tta[0])*sin(cur_tta[1]);
//		Jacob(3,4) = - cos(cur_tta[2])*sin(cur_tta[0]) - cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]);
//		Jacob(3,5) = cos(cur_tta[0])*cos(cur_tta[3])*sin(cur_tta[1]) - sin(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2]));
//		Jacob(3,6) = sin(cur_tta[4])*(cos(cur_tta[3])*(sin(cur_tta[0])*sin(cur_tta[2]) - cos(cur_tta[0])*cos(cur_tta[1])*cos(cur_tta[2])) + cos(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3])) - cos(cur_tta[4])*(cos(cur_tta[2])*sin(cur_tta[0]) + cos(cur_tta[0])*cos(cur_tta[1])*sin(cur_tta[2]));
//
//		Jacob(4,0) = 0;
//		Jacob(4,1) = 0;
//		Jacob(4,2) = cos(cur_tta[0]);
//		Jacob(4,3) = sin(cur_tta[0])*sin(cur_tta[1]);
//		Jacob(4,4) = cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2]);
//		Jacob(4,5) = sin(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) + cos(cur_tta[3])*sin(cur_tta[0])*sin(cur_tta[1]);
//		Jacob(4,6) = cos(cur_tta[4])*(cos(cur_tta[0])*cos(cur_tta[2]) - cos(cur_tta[1])*sin(cur_tta[0])*sin(cur_tta[2])) - sin(cur_tta[4])*(cos(cur_tta[3])*(cos(cur_tta[0])*sin(cur_tta[2]) + cos(cur_tta[1])*cos(cur_tta[2])*sin(cur_tta[0])) - sin(cur_tta[0])*sin(cur_tta[1])*sin(cur_tta[3]));
//
//		Jacob(5,0) = 1;
//		Jacob(5,1) = 1;
//		Jacob(5,2) = 0;
//		Jacob(5,3) = cos(cur_tta[1]);
//		Jacob(5,4) = sin(cur_tta[1])*sin(cur_tta[2]);
//		Jacob(5,5) = cos(cur_tta[1])*cos(cur_tta[3]) - cos(cur_tta[2])*sin(cur_tta[1])*sin(cur_tta[3]);
//		Jacob(5,6) = sin(cur_tta[4])*(cos(cur_tta[1])*sin(cur_tta[3]) + cos(cur_tta[2])*cos(cur_tta[3])*sin(cur_tta[1])) + cos(cur_tta[4])*sin(cur_tta[1])*sin(cur_tta[2]);

		//		// Kinematics
//		KDL::ChainFkSolverPos_recursive fkSolver = KDL::ChainFkSolverPos_recursive(RB7DOFKdl);
//		KDL::Frame fkKDL;
//		fkSolver.JntToCart(jnt_q, fkKDL);
		//std::cout << fkKDL << sep;
		//Trans_Jacob = Jacob.transpose();
		//pinv_Jacob = Jacob.completeOrthogonalDecomposition().pseudoInverse();
		//Tau = pinv_Jacob*pos_err;
		// Compute Dynamics
		KDL::Vector gravity(0.0, 0.0, -9.81);
		KDL::ChainIdSolver_RNE gcSolver = KDL::ChainIdSolver_RNE(RB7DOFKdl, gravity);
		gcSolver.CartToJnt(jnt_q, jnt_qd, jnt_qdd, jnt_wrenches,jnt_taugc);
		for (unsigned int i = 0; i < 7; i++)
		{
			GC_Tau(i,0) = jnt_taugc(i);
		}
//		pinv_Jacob = pseudoinverse(Jacob,0.001);

		tool_vel(0,0) = PosEE(0,0)- Pos_prev(0,0);
		tool_vel(1,0) = PosEE(1,0)- Pos_prev(1,0);
		tool_vel(2,0) = PosEE(2,0)- Pos_prev(2,0);
		tool_vel(3,0) = 0;
		tool_vel(4,0) = 0;
		tool_vel(5,0) = 0;
		Tau = Jacob.transpose()*(Stiffness*Pose_err - (Damping*tool_vel)/0.1);
		jointt_publish(Tau);
//		theta = theta + 0.5*pinv_Jacob*pos_err;
//		jointp_publish(theta);
		it = it+1;
		Pos_prev = PosEE;
//		std::cout << Tau << sep;

		//Jacobian matrix
		//joint to Pos Jacobian
		/*J0v7 =
					[ - d5*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - lg7*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - d3*sin(q1)*sin(q2), lg7*(cos(q6)*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q1)*cos(q2)*sin(q4) + cos(q1)*cos(q3)*cos(q4)*sin(q2)) - cos(q1)*sin(q2)*sin(q3)*sin(q5))) + d5*(cos(q1)*cos(q2)*cos(q4) - cos(q1)*cos(q3)*sin(q2)*sin(q4)) + d3*cos(q1)*cos(q2), lg7*(sin(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - cos(q6)*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - d5*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)), - lg7*(cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2))) - d5*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)),  lg7*sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))),  lg7*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))), 0]
					[   d3*cos(q1)*sin(q2) - lg7*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - d5*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)), lg7*(cos(q6)*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) + cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5))) + d5*(cos(q2)*cos(q4)*sin(q1) - cos(q3)*sin(q1)*sin(q2)*sin(q4)) + d3*cos(q2)*sin(q1), d5*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - lg7*(sin(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q6)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))),   lg7*(cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2))) + d5*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)), -lg7*sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))), -lg7*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))), 0]
					[                                                                                                                                                                                                                                                                                                                                                                   0,                                                                 lg7*(sin(q6)*(cos(q5)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4))) - d3*sin(q2) - d5*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)),                                                                                                                 lg7*(sin(q6)*(cos(q3)*sin(q2)*sin(q5) + cos(q4)*cos(q5)*sin(q2)*sin(q3)) + cos(q6)*sin(q2)*sin(q3)*sin(q4)) + d5*sin(q2)*sin(q3)*sin(q4),                                                                                                             - d5*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - lg7*(cos(q6)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))),                                                                  lg7*sin(q6)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)),                                                                                                     -lg7*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))), 0]
		 */
		//joint to Rot Jacobian
		/*J0w7 =  [0      -sin(q2)    -cos(q3)*sin(q2) -cos(q2)*sin(q4)-cos(q3)*cos(q4)*sin(q2)   sin(q2)*sin(q3)*sin(q5)-cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))   -cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))-sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4))  sin(q7)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))-cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)));
		         -1     0           -cos(q2)         sin(q2)*sin(q3)                            cos(q3)*sin(q2)*sin(q4)-cos(q2)*cos(q4)                                     sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3)                                                               cos(q7)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))-sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)))+cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4));
		         0      cos(q2)     sin(q2)*sin(q3)  cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)    sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3)   cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4))-sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))   0];
		*/




		//std::cout << pinv_Jacob << sep;

	//cout << cur_tta[1] << endl;
//	for (unsigned int i = 0; i < 7; i++)
//	{
//		printf("%f  \n",jnt_taugc(i));
//	}
//	printf("\n");
//	//printf("%f  \n",cur_tta[1]);
//	ros::Duration(0.01).sleep();
//	ros::spinOnce();

//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotZ)));
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
//	RB7DOFKdl.addSegment(Segment(Joint(Joint::RotZ)));


	// Create the frame that will contain the results
	//KDL::Frame cartpos;


	//KDL::RigidBodyInertia(1.0, KDL::Vector(-0.45, 0, 0), KDL::RotationalInertia(1, 1, 1, 0, 0, 0));
	//cout << inert << endl;
	//RB7DOFKdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.9, 0.0, 0.0, 0.0), inert));
	//RB7DOFKdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.9, 0.0, 0.1, 0.0), inert));
	}
	ros::Duration(0.01).sleep();
	ros::spinOnce();
	it = 0;

}
	return 0;
}
