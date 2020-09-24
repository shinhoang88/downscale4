#include <ctime> // time?
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>

#include "downscale4/one.h" // function??
#include "downscale4/two.h"
#include "downscale4/three.h"
#include "downscale4/seven.h"

#include  "downscale4/trans.h"
#include  "downscale4/buttons.h"

#include "ros/ros.h"
#include "ros/time.h"
#include </home/robotory/eigen/Eigen/Dense>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

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
#define lg7 0.12

//#define lg1 0
//#define lg3 0
//#define lg5 0
//#define lg7 0

//Joint mass (kg)
#define m1 6.82204
#define m2 5.77002
#define m3 5.22674
#define m4 3.63224
#define m5 2.72717
#define m6 2.14576
#define m7 0.25167

//#define m1 4
//#define m2 8.59206
//#define m3 4
//#define m4 4.85898
//#define m5 2
//#define m6 2.87293
//#define m7 0.25167

FILE *pData;


double SI,SI_COM,SI_HIG,SI_LOW,SI_FIX;
double SI_del=1;
double minT2;
double q[7];
double th6_prev;
double th6_del;

double Output_data[16];
int Buttons[2];
int Inkwell=0;
int button1_click_time = 0;
int button1_si_click_time = 0;
int button2_si_click_time = 0;
int count_num=0;

double hd_rotation_matrix[9];
double hd_current_xyz[3];
double hd_del_xyz[3];
double hd_initial_xyz[3];
double robot_initial_xyz[3];
double robot_current_xyz[3];
double robot_prev_xyz[3];
double robot_del_xyz[3];
double th[7];
double haptic_del_xyz[3];
double haptic_del_xyz_1[3];
double cur_vel[7];
double cur_vel_1[7];
double Euler_r, Euler_b, Euler_a;
double cur_tau[7];
float end_force[3];




using Eigen::MatrixXd;
using namespace std;

//  global parameters
const int	N = 1000; // 경로를 N등분 한것
double	dt;
//const int 	duration = 6;	//seconds



double		initial_pose_end_effector[3];	//x,y,z init postion for each trajectory _ computed by Forward Kinematics
double 		current_joint_states[7];		//current joint angles _ from  msgCallbackP function
double 		current_joint_torques[7];		//current joint torques _ from  msgCallbackT function
bool 		is_trajectory_possible;


ros::Publisher pub_jointp;
ros::Publisher pub_hap_del;
ros::Publisher pub_robot_del;
ros::Publisher pub_end_force;
ros::Subscriber sub_jointp;
ros::Subscriber sub_jointv;

//Given
MatrixXd P_INIT(4,4);	//Given init homogeneous transformation matrix
MatrixXd P_END(4,4);	//Given end homogeneous transformation matrix
MatrixXd P_COM(4,4);	//Given end homogeneous transformation matrix
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
MatrixXd Euler_x(3,3); 		//
MatrixXd Euler_y(3,3); 		//
MatrixXd Euler_z(3,3); 		//
MatrixXd Euler_angle(3,3); 		//


MatrixXd hd_initial_rotation(3,3); 		//
MatrixXd hd_del_rotation(3,3); 		//
MatrixXd haptic_del_rotation(3,3); 		//
MatrixXd haptic_del_rotation_1(3,3); 		//
MatrixXd hd_current_rotation(3,3); 		//
MatrixXd robot_initial_rotation(3,3); 		//
MatrixXd robot_current_rotation(3,3); 		//

MatrixXd T_07(4, 4);		//Forward kinematics

//by Forward Kinematics
MatrixXd RotMat(3, 3); 		//EE rotation matrix
MatrixXd PosEE(3, 1); 		//EE postion
MatrixXd FEE(3, 1); 		//End point forces matrix refer to EE frame 3 components
MatrixXd F0E(3, 1); 		//End point forces matrix refer to base frame

void DataSave(unsigned int count_num)
{
	pData = fopen("force_test.txt","a+");

	fprintf(pData, " %i", count_num);

	fprintf(pData, " %f", end_force[0]);
	fprintf(pData, " %f", end_force[1]);
	fprintf(pData, " %f", end_force[2]);
	/*
	fprintf(pData, "%f", count)
	fprintf(pData, "%f", count)
	fprintf(pData, "%f", count)
	fprintf(pData, "%f", count)
	fprintf(pData, "%f", count)
	fprintf(pData, "%f", count)
	*/

	fprintf(pData,"\n");
	fclose(pData);
}


void hd_callback_trans(const downscale4::trans::ConstPtr& msg) {

		Output_data[0] = msg->a;
		Output_data[1] = msg->b;
		Output_data[2] = msg->c;
		Output_data[3] = msg->d;
		Output_data[4] = msg->e;
		Output_data[5] = msg->f;
		Output_data[6] = msg->g;
		Output_data[7] = msg->h;
		Output_data[8] = msg->i;
		Output_data[9] = msg->j;
		Output_data[10] = msg->k;
		Output_data[11] = msg->l;
		Output_data[12] = msg->m;
		Output_data[13] = msg->n;
		Output_data[14] = msg->o;
		Output_data[15] = msg->p;

		/*
		hd_rotation_matrix[0] = Output_data[0];
		hd_rotation_matrix[1] = Output_data[1];
		hd_rotation_matrix[2] = Output_data[2];
		hd_rotation_matrix[3] = Output_data[4];
		hd_rotation_matrix[4] = Output_data[5];
		hd_rotation_matrix[5] = Output_data[6];
		hd_rotation_matrix[6] = Output_data[8];
		hd_rotation_matrix[7] = Output_data[9];
		hd_rotation_matrix[8] = Output_data[10];
		*/

		hd_rotation_matrix[0] = -Output_data[2];
		hd_rotation_matrix[1] = -Output_data[0];
		hd_rotation_matrix[2] = Output_data[1];
		hd_rotation_matrix[3] = -Output_data[6];
		hd_rotation_matrix[4] = -Output_data[4];
		hd_rotation_matrix[5] = Output_data[5];
		hd_rotation_matrix[6] = -Output_data[10];
		hd_rotation_matrix[7] = -Output_data[8];
		hd_rotation_matrix[8] = Output_data[9];


		//hd_current_xyz[0] = -Output_data[14];
		//hd_current_xyz[1] = -Output_data[12];
		//hd_current_xyz[2] = Output_data[13];

		hd_current_xyz[0] = Output_data[12];
		hd_current_xyz[1] = Output_data[13];
		hd_current_xyz[2] = Output_data[14];






}


void hd_callback_buttons(const downscale4::buttons::ConstPtr& msga) {

	Buttons[0] = msga->a;
	Buttons[1] = msga->b;
	Inkwell    = msga->c;


	//std::cout << std::string(80, '-') << std::endl;



}

//*********************Call Back***********************************
void msgCallbackP(const downscale4::seven::ConstPtr& msg)
{

	current_joint_states[0] = msg->a;
	current_joint_states[1] = msg->b;
	current_joint_states[2] = msg->c;
	current_joint_states[3] = msg->d;
	current_joint_states[4] = msg->e;
	current_joint_states[5] = msg->f;
	current_joint_states[6] = msg->g;

	th[0] = current_joint_states[0];
	th[1] = current_joint_states[1];
	th[2] = current_joint_states[2];
	th[3] = current_joint_states[3];
	th[4] = current_joint_states[4];
	th[5] = current_joint_states[5];
	th[6] = current_joint_states[6];


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

void forwd7(Eigen::MatrixXd &rot, Eigen::MatrixXd &pos, double th[7])
{

double th1, th2, th3, th4, th5, th6, th7;

	//Update theta from msgCallbackP function
	th1 = th[0];
	th2 = th[1];
	th3 = th[2];
	th4 = th[3];
	th5 = th[4];
	th6 = th[5];
	th7 = th[6];

rot(0, 0) = cos(th[6])*(sin(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*cos(th[3])*sin(th[1])) - cos(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])))) + sin(th[6])*(sin(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])));
	rot(0, 1) = cos(th[6])*(sin(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))) - sin(th[6])*(sin(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*cos(th[3])*sin(th[1])) - cos(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))));
	rot(0, 2) = -cos(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*cos(th[3])*sin(th[1])) - sin(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])));

	rot(1, 0) = -cos(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + cos(th[3])*sin(th[0])*sin(th[1])) - cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])))) - sin(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])));
	rot(1, 1) = sin(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + cos(th[3])*sin(th[0])*sin(th[1])) - cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])))) - cos(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])));
	rot(1, 2) = cos(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + cos(th[3])*sin(th[0])*sin(th[1])) + sin(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])));

	rot(2, 0) = sin(th[6])*(sin(th[4])*(cos(th[1])*sin(th[3]) + cos(th[2])*cos(th[3])*sin(th[1])) + cos(th[4])*sin(th[1])*sin(th[2])) - cos(th[6])*(cos(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) + cos(th[2])*cos(th[3])*sin(th[1])) - sin(th[1])*sin(th[2])*sin(th[4])) + sin(th[5])*(cos(th[1])*cos(th[3]) - cos(th[2])*sin(th[1])*sin(th[3])));
	rot(2, 1) = cos(th[6])*(sin(th[4])*(cos(th[1])*sin(th[3]) + cos(th[2])*cos(th[3])*sin(th[1])) + cos(th[4])*sin(th[1])*sin(th[2])) + sin(th[6])*(cos(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) + cos(th[2])*cos(th[3])*sin(th[1])) - sin(th[1])*sin(th[2])*sin(th[4])) + sin(th[5])*(cos(th[1])*cos(th[3]) - cos(th[2])*sin(th[1])*sin(th[3])));
	rot(2, 2) = cos(th[5])*(cos(th[1])*cos(th[3]) - cos(th[2])*sin(th[1])*sin(th[3])) - sin(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) + cos(th[2])*cos(th[3])*sin(th[1])) - sin(th[1])*sin(th[2])*sin(th[4]));

	pos(0, 0) = d3 * cos(th[0])*sin(th[1]) - d7 * (cos(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*cos(th[3])*sin(th[1])) + sin(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])))) - d5 * (sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*cos(th[3])*sin(th[1]));
	pos(1, 0) = d5 * (sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + cos(th[3])*sin(th[0])*sin(th[1])) + d7 * (cos(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + cos(th[3])*sin(th[0])*sin(th[1])) + sin(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])))) + d3 * sin(th[0])*sin(th[1]);
	pos(2, 0) = d1 - d7 * (sin(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) + cos(th[2])*cos(th[3])*sin(th[1])) - sin(th[1])*sin(th[2])*sin(th[4])) - cos(th[5])*(cos(th[1])*cos(th[3]) - cos(th[2])*sin(th[1])*sin(th[3]))) + d5 * (cos(th[1])*cos(th[3]) - cos(th[2])*sin(th[1])*sin(th[3])) + d3 * cos(th[1]);


	robot_current_xyz[0] = d3 * cos(th1)*sin(th2) - d7 * (cos(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))) - d5 * (sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2));

	robot_current_xyz[1] = d5 * (sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) + d7 * (cos(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) + d3 * sin(th1)*sin(th2);

	robot_current_xyz[2] = d1 - d7 * (sin(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) - cos(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))) + d5 * (cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4)) + d3 * cos(th2);
}



/*
void msgCallbackT(const downscale4::seven::ConstPtr& msg)
{

	current_joint_torques[0] = msg->a;
	current_joint_torques[1] = msg->b;
	current_joint_torques[2] = msg->c;
	current_joint_torques[3] = msg->d;
	current_joint_torques[4] = msg->e;
	current_joint_torques[5] = msg->f;
	current_joint_torques[6] = msg->g;

}
*/
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


//****************************Publishing message*********************
void joint_publish(double theta[7])
{
	downscale4::seven msgp;
	msgp.a = theta[0];
	msgp.b = theta[1];
	msgp.c = theta[2];
	msgp.d = theta[3];
	msgp.e = theta[4];
	msgp.f = theta[5];
	msgp.g = theta[6];

	pub_jointp.publish(msgp);

}

void hap_del_publish(double haptic_del_xyz[3])
{
	downscale4::three msgd;
	msgd.a = haptic_del_xyz[0]*0.001;
	msgd.b = haptic_del_xyz[1]*0.001;
	msgd.c = haptic_del_xyz[2]*0.001;


	pub_hap_del.publish(msgd);

}

void robot_del_publish(double robot_current_xyz[3])
{
	downscale4::three msgr;
	msgr.a = robot_current_xyz[0];
	msgr.b = robot_current_xyz[1];
	msgr.c = robot_current_xyz[2];


	pub_robot_del.publish(msgr);

}

void end_force_publish(float end_force[3])
{
	downscale4::three msge;
	msge.a = end_force[0];
	msge.b = end_force[0];
	msge.c = end_force[0];


	pub_end_force.publish(msge);

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


	cur_vel_1[0] = cur_vel[0]*(60/(2*pi));
	cur_vel_1[1] = cur_vel[1]*(60/(2*pi));
	cur_vel_1[2] = cur_vel[2]*(60/(2*pi));
	cur_vel_1[3] = cur_vel[3]*(60/(2*pi));
	cur_vel_1[4] = cur_vel[4]*(60/(2*pi));
	cur_vel_1[5] = cur_vel[5]*(60/(2*pi));
	cur_vel_1[6] = cur_vel[6]*(60/(2*pi));
}



//*********************main****************************************
int main(int argc, char **argv)

{

	ros::init(argc, argv, "teleoperation_downscale");  // Node name initialization
	ros::NodeHandle nh;                            // Node handle declaration for communication with ROS

	pub_jointp = nh.advertise<downscale4::seven>("gazebo/downscale_jointp", 100);
	pub_hap_del = nh.advertise<downscale4::three>("downscale_hap_del", 100);
	pub_robot_del = nh.advertise<downscale4::three>("downscale_robot_del", 100);
	pub_end_force = nh.advertise<downscale4::three>("end_force", 100);


	ros::Subscriber hd_trans = nh.subscribe("/hd_trans", 1, &hd_callback_trans);
	ros::Subscriber hd_buttons = nh.subscribe("/hd_buttons", 1, &hd_callback_buttons);
	ros::Subscriber sub_jointp;
	ros::Subscriber sub_jointv;
	ros::Subscriber sub_jointt;
	sub_jointp = nh.subscribe("downscale_actp", 100, msgCallbackP);
	sub_jointv = nh.subscribe("downscale_actv", 100, msgCallbackv);
	sub_jointv = nh.subscribe("downscale_actt", 100, msgCallbackT);





	//Given init homogeneous transformation matrix
	/*
	P_INIT << 	1, 	0.9211, 	0.3894, 	0,
                        0, 	-0.9211, 	-0.3894, 	0.5,
                        1, 	0.3894, 	-0.9211, 	0.5,
                        0, 	0, 		0, 		1;
	*/

	P_INIT << 	1, 	 0, 	 0, 	  0,
                0, 	-1, 	 0, 	0.5,
                0, 	 0, 	-1, 	0.5,
                0, 	 0, 	 0, 	  1;

	robot_initial_xyz[0] = P_INIT(0,3);
	robot_initial_xyz[1] = P_INIT(1,3);
	robot_initial_xyz[2] = P_INIT(2,3);

	//Given Arm angle

	//SI_FIX = -0.7;
	//SI_FIX = 1+SI_del;



//////////////////////////////////////////////////////////////////////////////////////////





	robot_initial_rotation <<		P_INIT(0,0), 	P_INIT(0,1), 	P_INIT(0,2),
        							P_INIT(1,0), 	P_INIT(1,1), 	P_INIT(1,2),
                					P_INIT(2,0), 	P_INIT(2,1), 	P_INIT(2,2);



	robot_current_rotation = robot_initial_rotation;

	hd_del_rotation <<			1, 0, 0,
								0, 1, 0,
								0, 0, 1;



//////////////////////////////////////////////////////////////////////////////////////////




	//(2).Compute Joint Trajectory q[7] for each step
	P_COM = P_INIT;


	//(i).Initial position step[0]
	InvK7(P_COM, SI_del);				//compute Inverse Kinematics, return values is scalar vector q[7]

	//printf("%f \t %f \t %f \t %f \n", q[0], q[1], q[2], q[3]);

	/*
	joint_publish(q);				//publish theta message
	ros::Duration(0.01).sleep();
	ros::spinOnce();
*/

	//T2_Opt(SI_LOW, SI_HIG);

	//printf("Finished \n");
	ros::Rate rate(1000);
	while (ros::ok())
	{
	//Given end homogeneous transformation matrix

	robot_prev_xyz[0]=robot_current_xyz[0];
	robot_prev_xyz[1]=robot_current_xyz[1];
	robot_prev_xyz[2]=robot_current_xyz[2];


	th6_prev=th[5];
		joint_publish(q);				//remain the destination position
		hap_del_publish(haptic_del_xyz);
		robot_del_publish(robot_current_xyz);
		end_force_publish(end_force);
		//ros::Duration(0.001).sleep();
		ros::spinOnce();


	robot_del_xyz[0]=robot_current_xyz[0]-robot_prev_xyz[0];
	robot_del_xyz[1]=robot_current_xyz[1]-robot_prev_xyz[1];
	robot_del_xyz[2]=robot_current_xyz[2]-robot_prev_xyz[2];

	forwd7(RotMat, PosEE, th);
	FEE(0, 0) = cur_tau[0]; 	//Fx
	FEE(1, 0) = cur_tau[1]; 	//Fy
	FEE(2, 0) = cur_tau[2];	//Fz
	F0E = RotMat * FEE;
	std::cout<<std::string(80,'-')<<std::endl;
	printf("%f \t %f \t %f  \n", F0E(0,0), F0E(1,0), F0E(2,0));

	end_force[0]=F0E(0,0);
	end_force[1]=F0E(1,0);
	end_force[2]=F0E(2,0);
	printf("%f \t %f \t %f  \n", end_force[0], end_force[1], end_force[2]);




	hd_current_rotation <<		hd_rotation_matrix[0], 	hd_rotation_matrix[3], 	hd_rotation_matrix[6],
    							hd_rotation_matrix[1], 	hd_rotation_matrix[4], 	hd_rotation_matrix[7],
       							hd_rotation_matrix[2], 	hd_rotation_matrix[5], 	hd_rotation_matrix[8];








		if (Buttons[0] == 1 && Inkwell == 1)
		{
			if (button1_click_time == 0)
			{
				hd_initial_xyz[0] = hd_current_xyz[0];
				hd_initial_xyz[1] = hd_current_xyz[1];
				hd_initial_xyz[2] = hd_current_xyz[2];
				//robot_initial_xyz[0] = robot_current_xyz[0];
				//robot_initial_xyz[1] = robot_current_xyz[1];
				//robot_initial_xyz[2] = robot_current_xyz[2];


				hd_initial_rotation <<		hd_rotation_matrix[0], 	hd_rotation_matrix[3], 	hd_rotation_matrix[6],
        									hd_rotation_matrix[1], 	hd_rotation_matrix[4], 	hd_rotation_matrix[7],
                							hd_rotation_matrix[2], 	hd_rotation_matrix[5], 	hd_rotation_matrix[8];

				//hd_initial_rotation = hd_del_rotation;

			}
			hd_del_xyz[0] = hd_current_xyz[0] - hd_initial_xyz[0];
			hd_del_xyz[1] = hd_current_xyz[1] - hd_initial_xyz[1];
			hd_del_xyz[2] = hd_current_xyz[2] - hd_initial_xyz[2];
			haptic_del_xyz[0] = haptic_del_xyz[0] - hd_del_xyz[2];
			haptic_del_xyz[1] = haptic_del_xyz[1] - hd_del_xyz[0];
			haptic_del_xyz[2] = haptic_del_xyz[2] + hd_del_xyz[1];
			hd_initial_xyz[0] = hd_current_xyz[0];
			hd_initial_xyz[1] = hd_current_xyz[1];
			hd_initial_xyz[2] = hd_current_xyz[2];

			hd_del_rotation = hd_current_rotation*hd_initial_rotation.transpose();
			//haptic_del_rotation = oil*hd_del_rotation;
			//rotation matrix to Euler angle

			Euler_b=(-asin(hd_del_rotation(2,0)))/1;
			if (Euler_b != 90*pi/180 && Euler_b != -90*pi/180)
			{
				Euler_a=(atan2(hd_del_rotation(1,0)/cos(Euler_b),hd_del_rotation(0,0)/cos(Euler_b)))/1;
				Euler_r=(atan2(hd_del_rotation(2,1)/cos(Euler_b),hd_del_rotation(2,2)/cos(Euler_b)))/1;
			}
			else if (Euler_b == 90*pi/180)
			{
				Euler_b=(90*pi/180)/1;
				Euler_a=0/1;
				Euler_r=(atan2(hd_del_rotation(0,1),hd_del_rotation(1,1)))/1;
			}
			else
			{
				Euler_b=(-90*pi/180)/1;
				Euler_a=0/1;
				Euler_r=(-atan2(hd_del_rotation(0,1),hd_del_rotation(1,1)))/1;
			}

			Euler_x << 	1, 			   0, 				0,
                    	0, 	cos(Euler_r), 	-sin(Euler_r),
                    	0, 	sin(Euler_r), 	cos(Euler_r);

			Euler_y << 	cos(Euler_b),  0, 	 sin(Euler_b),
                    	0, 			   1,			 	0,
                    	-sin(Euler_b), 0, 	 cos(Euler_b);

			Euler_z << 	cos(Euler_a), -sin(Euler_a),	0,
                    	sin(Euler_a),  cos(Euler_a), 	0,
                    	0, 						  0, 	1;

			Euler_angle=Euler_z*Euler_y*Euler_x;

			robot_current_rotation = Euler_angle*robot_current_rotation;
			hd_initial_rotation = hd_current_rotation;

			button1_click_time++;




		}
		else {
			//robot_initial_xyz[0] = robot_current_xyz[0];
			//robot_initial_xyz[1] = robot_current_xyz[1];
			//robot_initial_xyz[2] = robot_current_xyz[2];
			button1_click_time = 0;
			hd_del_xyz[0] = 0;
			hd_del_xyz[1] = 0;
			hd_del_xyz[2] = 0;

			hd_del_rotation <<		1, 	0, 	0,
        							0, 	1, 	0,
                					0, 	0, 	1;

			//haptic_del_xyz[0] = haptic_del_xyz[0] - hd_del_xyz[2];
			//haptic_del_xyz[1] = haptic_del_xyz[1] - hd_del_xyz[0];
			//haptic_del_xyz[2] = haptic_del_xyz[2] + hd_del_xyz[1];
		}

		if (Buttons[0] == 1 && Inkwell == 0)
		{
			if (button1_si_click_time == 0)
			{
				SI_del = SI_del + 0.00005;

			}

			//button1_si_click_time++;
		}
		else {
			button1_si_click_time = 0;
		}




		if (Buttons[1] == 1 && Inkwell == 0)
		{
			if (button2_si_click_time == 0)
			{
				SI_del = SI_del - 0.00005;

			}

			//button2_si_click_time++;

		}
		else {
			button2_si_click_time = 0;
		}
		/*
		std::cout<<std::string(80,'-')<<std::endl;
		printf("%f \t %f \t %f \t %f \n", hd_rotation_matrix[0], hd_rotation_matrix[3], hd_rotation_matrix[6], haptic_del_xyz[0]);
		printf("%f \t %f \t %f \t %f \n", hd_rotation_matrix[1], hd_rotation_matrix[4], hd_rotation_matrix[7], haptic_del_xyz[1]);
		printf("%f \t %f \t %f \t %f \n", hd_rotation_matrix[2], hd_rotation_matrix[5], hd_rotation_matrix[8], haptic_del_xyz[2]);
		printf("%f \t %f \t %f \t %f \n", Output_data[3], Output_data[7], Output_data[11], Output_data[15]);
		printf("%d \n", button1_click_time);
		printf("%d \t %d \t %d \n", Buttons[0], Buttons[1], Inkwell);
		*/

		/*
		std::cout<<std::string(80,'-')<<std::endl;
		printf("%f \t %f \t %f  \n", hd_rr(0,0), hd_rr(0,1), hd_rr(0,2));
		printf("%f \t %f \t %f  \n", hd_rr(1,0), hd_rr(1,1), hd_rr(1,2));
		printf("%f \t %f \t %f  \n", hd_rr(2,0), hd_rr(2,1), hd_rr(2,2));
		*/

		/*
		std::cout<<std::string(80,'-')<<std::endl;
		printf("%f \t %f \t %f  \n", hd_initial_rotation(0,0), hd_initial_rotation(0,1), hd_initial_rotation(0,2));
		printf("%f \t %f \t %f  \n", hd_initial_rotation(1,0), hd_initial_rotation(1,1), hd_initial_rotation(1,2));
		printf("%f \t %f \t %f  \n", hd_initial_rotation(2,0), hd_initial_rotation(2,1), hd_initial_rotation(2,2));
		printf("%f \t %f \t %f  \n", hd_del_rotation(0,0), hd_del_rotation(0,1), hd_del_rotation(0,2));
		printf("%f \t %f \t %f  \n", hd_del_rotation(1,0), hd_del_rotation(1,1), hd_del_rotation(1,2));
		printf("%f \t %f \t %f  \n", hd_del_rotation(2,0), hd_del_rotation(2,1), hd_del_rotation(2,2));
		*/
/*
		std::cout<<std::string(80,'-')<<std::endl;
		printf("%f \t %f \t %f  \n", robot_current_rotation(0,0), robot_current_rotation(0,1), robot_current_rotation(0,2));
		printf("%f \t %f \t %f  \n", robot_current_rotation(1,0), robot_current_rotation(1,1), robot_current_rotation(1,2));
		printf("%f \t %f \t %f  \n", robot_current_rotation(2,0), robot_current_rotation(2,1), robot_current_rotation(2,2));
		printf("%f \t %f \t %f  \n", hd_del_rotation(0,0), hd_del_rotation(0,1), hd_del_rotation(0,2));
		printf("%f \t %f \t %f  \n", hd_del_rotation(1,0), hd_del_rotation(1,1), hd_del_rotation(1,2));
		printf("%f \t %f \t %f  \n", hd_del_rotation(2,0), hd_del_rotation(2,1), hd_del_rotation(2,2));

	//printf("%f \t %f \t %f \n", th[0], th[1], th[2]);
	printf("%f \t %f \t %f \t \n", robot_initial_xyz[0] + haptic_del_xyz[0]*0.003, robot_initial_xyz[1] + haptic_del_xyz[1]*0.003, robot_initial_xyz[2] + haptic_del_xyz[2]*0.003);
	printf("%f \t %f \t %f \n", robot_current_xyz[0], robot_current_xyz[1], robot_current_xyz[2]);
	printf("%f \t %f \t %f  \n", cur_vel[0], cur_vel[1], cur_vel[2]);
	printf("%f \t %f \t %f  \n", cur_vel[3], cur_vel[5], cur_vel_1[5]);
*/
//A=A+0.3 + haptic_del_xyz[0]*0.01
	/*
	P_END << 	1, 	0.9211, 	0.3894, 	robot_initial_xyz[0] + haptic_del_xyz[0]*0.0001,
                        0, 	-0.9211, 	-0.3894, 	robot_initial_xyz[1] + haptic_del_xyz[1]*0.0001,
                        1, 	0.3894, 	-0.9211, 	robot_initial_xyz[2] + haptic_del_xyz[2]*0.0001,
                        0, 	0, 		0, 		1;
	*/


	P_END << 	robot_current_rotation(0,0), 	robot_current_rotation(0,1), 	robot_current_rotation(0,2), 	robot_initial_xyz[0] + haptic_del_xyz[0]*0.003,
                robot_current_rotation(1,0), 	robot_current_rotation(1,1), 	robot_current_rotation(1,2), 	robot_initial_xyz[1] + haptic_del_xyz[1]*0.003,
                robot_current_rotation(2,0), 	robot_current_rotation(2,1), 	robot_current_rotation(2,2), 	robot_initial_xyz[2] + haptic_del_xyz[2]*0.003,
                0, 														  0, 						   	  0, 												  1;


	/*
	P_END << 			P_INIT(0,0), 	P_INIT(0,1), 	P_INIT(0,2), 	robot_initial_xyz[0] + haptic_del_xyz[0]*0.001,
                        P_INIT(1,0), 	P_INIT(1,1), 	P_INIT(1,2), 	robot_initial_xyz[1] + haptic_del_xyz[1]*0.001,
                        P_INIT(2,0), 	P_INIT(2,1), 	P_INIT(2,2), 	robot_initial_xyz[2] + haptic_del_xyz[2]*0.001,
                        0, 	0, 		0, 		1;
	*/
	P_COM = P_END;

		InvK7(P_COM, SI_del);


		//end = clock();
	th6_del = (th[5]-th6_prev)*(60/(2*pi))/0.001;//*(60/(2*pi))*160;




	DataSave(count_num);
	count_num ++;
	//printf("%f  \n", SI_del);
	//rate.sleep();

	}



	return 0;
}
