//****************************************INCLUDING***************************************************
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include "downscale4/one.h" 
#include "downscale4/two.h" 
#include "downscale4/three.h" 
#include "downscale4/seven.h"
#include "downscale4/trans.h"
#include "downscale4/buttons.h"
#include "downscale4/charmsg.h"

#include "ros/ros.h"
#include "ros/time.h"

#include </home/nuclear/eigen/Eigen/Dense>
#include </home/nuclear/eigen/pseudoinverse.cpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
//#include "PowerPMACcontrol.h"
//#include "argParser.h"

//************************************GLOBAL DEFINITION**********************************************
#define pi 3.14159265359
#define g_acc 9.81


using namespace std;
//using namespace PowerPMACcontrol_ns;
using Eigen::MatrixXd;
//*************************************ARM CONFIGURATION**********************************************
//Joint offset [m]
#define d1 0.371
#define d3 0.547
#define d5 0.484
//#define d7 0.273
#define d7 0.323
//Arm offset [m] - Issac dynamics Eq
#define L1 0.371
#define L3 0.547
#define L5 0.484
#define L7 0.273
#define L2 0.547
#define L4 0.484
//#define L6 0.273
#define L6 0.323

//Center of mass for each joint, from local coordinate [m]
#define lg1 0.247
#define lg2 0.32328
#define lg3 0.24043
#define lg4 0.52448
#define lg5 0.21711
#define lg6 0.46621
#define lg7 0.15735
#define lge 0.02301
//Center of mass for each joint, from local coordinate (m)
double LL1=L1-lg1;
double LL3=L3-lg3;
double LL5=L5-lg5;
double LL7=L7-lg7;

double LL2=-L3+lg3;
double LL4=-L5+lg5;
double LL6=-L7+lg7;

//External force at EE
double fx, fy, fz, falp, fbet, fgam;

//Positive weigting matrix used to give priority to joints with higher torque capacity/better mechanical advantage
double K1, K2, K3, K4, K5, K6, K7;



    
    
//Gear ratio
#define kr1 1   //160
#define kr2 1   //160
#define kr3 1   //160
#define kr4 1   //160
#define kr5 1   //160
#define kr6 1   //160
#define kr7 1   //160

//Motor inertia
double im1=0.24*0.0001;
double im1zz=0.24*0.0001;

double im2=1.447*0.0001;
double im2zz=1.447*0.0001;

double im3=0.437*0.0001;
double im3zz=0.437*0.0001;

double im4=0.437*0.0001;
double im4zz=0.437*0.0001;

double im5=0.24*0.0001;
double im5zz=0.24*0.0001;

double im6=0.24*0.0001;
double im6zz=0.24*0.0001;

double im7=0.0303*0.0001;
double im7zz=0.0303*0.0001;

//inertia for dyn
double ia1xx=0.2558; 
double ia1yy=0.1516;
double ia1zz=0.1804;
double ia1xy=0;
double ia1xz=0;
double ia1yz=0;

double ia2xx=0.2235; 
double ia2yy=0.1796;
double ia2zz=0.0707;
double ia2xy=0;
double ia2xz=0;
double ia2yz=0.0041;

double ia3xx=0.1651; 
double ia3yy=0.0868;
double ia3zz=0.1268;
double ia3xy=0;
double ia3xz=0;
double ia3yz=0;

double ia4xx=0.097; 
double ia4yy=0.0768;
double ia4zz=0.03;
double ia4xy=0;
double ia4xz=0;
double ia4yz=-0.0025;

double ia5xx=0.0228; 
double ia5yy=0.003;
double ia5zz=0.0151;
double ia5xy=0;
double ia5xz=0;
double ia5yz=0;

double ia6xx=0.0371; 
double ia6yy=0.0288;
double ia6zz=0.0141;
double ia6xy=0;
double ia6xz=0;
double ia6yz=0;

double ia7xx=0.0001; 
double ia7yy=0.0001;
double ia7zz=0.0001;
//double ia7xx=0.01; 
//double ia7yy=0.01;
//double ia7zz=0.01;
double ia7xy=0;
double ia7xz=0;
double ia7yz=0;

//following modelling world file
/* 
double m1 = 23;
double m2 = 15;
double m3 = 24;
double m4 = 6.3;
double m5 = 6.74;
double m6 = 3.8;
//double m7 = 0.808; // 3.8 + eef 
double m7 = 0.808; // 3.8 + eef 
*/

double m1 = 23;
double m2 = 16.5;
double m3 = 25.5;
double m4 = 6.4;
double m5 = 6.74;
double m6 = 3.9;
//double m7 = 0.808; // 3.8 + eef 
double m7 = 1; // 3.8 + eef 

//------------------data save -------------------------------------------

FILE *pData;




//Joint mass (kg)
double m[7];

std::string sep = "\n----------------------------------------\n";
//***********************************MASTER DEVICE SUBCRIBER******************************************
double Output_data[16];             //4x4 homogeneous trans from master device - hd_callback_trans
double hd_rotation_matrix[9];       //3x3 rotation matrix coverted from Output_data - hd_callback_trans
double hd_current_xyz[3];           //current position
double hd_del_xyz[3];               //position errors
double hd_initial_xyz[3];           //init position
double robot_prev_xyz[3];           //previous position
double robot_del_xyz[3];            //Differential position
double robot_initial_xyz[3];        //Init EE position
double haptic_del_xyz[3];

int Buttons[2];                     //Phantom omni button variables - hd_callback_buttons
int Inkwell=0;                      //Phantom omni Inkwell button - hd_callback_buttons
int button0_click_time = 0;         //Buttons[0] clicked counting variable
int button1_click_time = 0;
double Euler_r, Euler_b, Euler_a;
double time_taken = 0.003;

MatrixXd robot_initial_rotation(3,3); 		//
MatrixXd robot_current_rotation(3,3); 		//
MatrixXd hd_del_rotation(3,3); 		//
MatrixXd hd_current_rotation(3,3); 		//
MatrixXd hd_initial_rotation(3,3); 		//
MatrixXd Euler_angle(3,3); 		//
MatrixXd Euler_x(3,3); 		//
MatrixXd Euler_y(3,3); 		//
MatrixXd Euler_z(3,3); 		//

char ReadKey;


//**************************************PMAC DEFINITION***********************************************
int motor[7];                           //motor index
double Kt[7];                           //motor torque constant (1/A)
double Nhd[7];                          //Harmonic driver's gear ratio
double Icon[7];                         //motor driver's continuous current (A)
double Kp[7], Kd[7];	                //P gain for joint i
double T_limit[7];	                    //Torque limit (for safety)
double fric_amp_lim[7];                 //Friction compensate current (A) limit +/-




//**************************************GLOBAL PARAMETERS*********************************************
//ROSE nodes description:
ros::Publisher pub_jointp;	                        //joint position publisher node
ros::Publisher pub_jointt;	                        //joint torque publisher node
ros::Subscriber hd_trans;                           //sub transformation matrix
ros::Subscriber hd_buttons;                         //sub phantom omni button
ros::Subscriber hd_char;	                        //sub keyboard
ros::Subscriber sub_jointp;	                        //joint position subcriber node
ros::Subscriber sub_jointv;	                        //joint velocity subcriber node
ros::Subscriber sub_jointt;	                        //joint torque subcriber node


double	current_joint_states[7];	                //current joint angles _ from  msgCallbackP function
double	current_joint_vel[7];	                //current joint angles _ from  msgCallbackP function
double	cur_vel[7];			                        //current joint vel _ from  msgCallbackv function
double	cur_tau[7];			                        //current joint torque _ from  msgCallbackT function
double	robot_current_xyz[3];;		                //current pos - output of forward kinematics

double SI,SI_COM,SI_HIG,SI_LOW,SI_FIX;              //Computed arm angles - redundancy
double q[7], q_pmac[7], th[7], q_pmac_update[7], dq_pmac_update[7]; //Imtermediate joint angles for a specific computational
const int N = 1000;		                            //Number of Cartesian steps
double	dt;			                                //incremental step time

//********************************Eigen Matrices Definition***********************************
//Given
MatrixXd P_INIT(4,4);	            //Given init homogeneous transformation matrix
MatrixXd P_END(4,4);	            //Given end homogeneous transformation matrix
MatrixXd P_COM(4,4);	            //Given end homogeneous transformation matrix

//Temp matrices for Inverse Kinematics computation
MatrixXd W(3,1); 		            //Wrist position relating to base
MatrixXd DZ(3,1);		            //[0; 0; 1] matrix
MatrixXd L_SW(3,1); 	            //Wrist position relating to shoulder
MatrixXd D1(3,1); 		            //D1 column matrix
MatrixXd KSW(3,3); 		            //Skew symetric matrix of vector between wrist and shoulder
MatrixXd USW(3,1); 		            //Unit vector of L_SW
MatrixXd R03dot(3,3); 		        //Rotation matrix of the ref joint angles q1dot, q2dot
MatrixXd XS(3,3); 		            //Shoulder constant matrix
MatrixXd YS(3,3); 		            //
MatrixXd ZS(3,3); 		            //
MatrixXd I(3,3); 		            //Identity matrix
MatrixXd I7(7,7); 		            //Identity matrix 7x7
MatrixXd R34(3,3); 		            //Rotation matrix from joint 3 to joint 4
MatrixXd XW(3,3); 		            //Wrist constant matrix
MatrixXd YW(3,3); 		            //
MatrixXd ZW(3,3); 		            //

//Temp matrices for stiffness control computation
MatrixXd desired_theta(7,1);	    //Desired joint positions
MatrixXd desired_dtheta(7,1);	    //Desired joint velocities
MatrixXd current_theta(7,1);	    //Current joint position
MatrixXd current_dtheta(7,1);	    //Current joint velocities
MatrixXd theta_err1(7,1);	        //Joint position error 1
MatrixXd theta_err2(7,1);	        //Joint position error 2
MatrixXd dtheta_err(7,1);	        //Joint velocity error

MatrixXd Jacob(6,7);		        //Jacobian matrix 6x7
MatrixXd DiffJacob(6,7);		    //Differentiate of Jacobian matrix 6x7
MatrixXd Trans_Jacob(7,6);	        //Jacobian transpose matrix 7x6
MatrixXd pinv_Jacob(7,6);	        //Jacobian pseudo inverse matrix 7x6 - unweighted pseudo inverse
MatrixXd Mpinv_Jacob(7,6);	        //Jacobian pseudo inverse matrix 7x6 - weighted pseudo inverse
MatrixXd Mpinv_Jacob_Prev(7,6);	        //Jacobian pseudo inverse matrix 7x6 - weighted pseudo inverse
MatrixXd pinv_Jacob_Prev(7,6);	        //Jacobian pseudo inverse matrix 7x6 - weighted pseudo inverse
MatrixXd Diff_Mpinv_Jacob(7,6);	    //Differentiate of Jacobian pseudo inverse matrix 7x6 - weighted pseudo inverse
MatrixXd Diff_pinv_Jacob(7,6);	    //Differentiate of Jacobian pseudo inverse matrix 7x6 - weighted pseudo inverse


MatrixXd Mass(7,7);		            //Mass matrix 7x7
MatrixXd Coriolis(7,1);		        //Coriolis and centrifugal matrix 7x1
MatrixXd Coriolis_k(7,1);		        //Coriolis and centrifugal matrix 7x1
MatrixXd Gravity(7,1);		        //Gravity compensate matrix 7x1

MatrixXd kd_matrix_7(7,7);
MatrixXd kp_matrix_7(7,7);

MatrixXd kd_matrix_6(6,6);
MatrixXd kp_matrix_6(6,6);

MatrixXd SCL_FACTOR(7,7);

MatrixXd K_prior(7,7);              //Positive weigting matrix used to give priority to joints with higher torque capacity/better mechanical advantage


MatrixXd Kx(6,6); 	                //cartesian stiffness diagonal matrix
MatrixXd KxVec(6,1); 	            //stiffness diagonal vector
MatrixXd Ki(7,7); 	                //joint stiffness matrix
MatrixXd Kmin(7,7); 	            //minimum joint stiffness matrix under mapping Kc -> Ki
MatrixXd Knull(7,7); 	            //Null space joint stiffness under mapping Kc -> Ki
MatrixXd K0(7,7); 	                //weighting matrix (symmetric positive definite) under mapping Kc -> Ki
MatrixXd K0Vec(7,1); 	            //weighting matrix diagonal vector

MatrixXd Dx(6,6); 	                //cartesian damping diagonal matrix
MatrixXd DxVec(6,1); 	            //damping diagonal vector
MatrixXd Di(7,7); 	                //joint damping matrix

MatrixXd Tau_stif(7,1);		        //Torque computed from stiffness control
MatrixXd Friction_I(7,1);	        //Friction compensation torque
MatrixXd Gravity_T(7,1);	        //Gravity compensation torque
MatrixXd Tau_I(7,1);		        //Overall torque -> send to PMAC
MatrixXd Tau_I_prev(7,1);		        //Overall torque -> send to PMAC
MatrixXd Tau_e(7,1);		        //End point torque 
MatrixXd Tau_null(7,1);
MatrixXd v7(6,1);		            //End point force
MatrixXd grad_cost(7,1);            //Gradient of torque minimization cost function

MatrixXd dy1(7,1);                  //secondary velocities (null space) for static torque minimization at current state
MatrixXd dy2(7,1);                  //secondary velocities (null space) for dynamic torque minimization at current state
MatrixXd dy(7,1);                   //SUM of secondary velocities (null space) for both static & dynamic torque minimization at current state
MatrixXd dy1_prev(7,1);             //secondary velocities (null space) for static torque minimization at previous state
MatrixXd dy2_prev(7,1);             //secondary velocities (null space) for dynamic torque minimization at previous state
MatrixXd ddy1(7,1);                 //secondary acceleration (null space) for static torque minimization at current state
MatrixXd ddy2(7,1);                 //secondary acceleration (null space) for dynamic torque minimization at current state
MatrixXd ddy2_prev(7,1);            //secondary acceleration (null space) for dynamic torque minimization at previous state
MatrixXd ddy(7,1);                  //SUM of secondary acceleration (null space) for both static & dynamic torque minimization at current state

double alpha1, alpha2;              //weighting factors for minimizing the static torque term & dynamic torque term, respectively 


MatrixXd e_Ndot(7,1);
MatrixXd phi_N(7,1);
MatrixXd dpd_prev(6,1);
MatrixXd ddy1_prev(7,1); 

MatrixXd jointpos(6,1);
MatrixXd currentp(6,1);
MatrixXd currentdp(6,1); 
MatrixXd pderror(6,1);
MatrixXd dpderror(6,1);
MatrixXd maxperror(3,1);

MatrixXd prevp(6,1);
MatrixXd a0(6,1);       //cubic polynomial coefficient
MatrixXd a1(6,1);       //cubic polynomial coefficient
MatrixXd a2(6,1);       //cubic polynomial coefficient
MatrixXd a3(6,1);       //cubic polynomial coefficient
MatrixXd p0(6,1);       //Init pose

MatrixXd pe(6,1);       //End pose
MatrixXd pd(6,1);       //calculated desired cartesian pose for each step
MatrixXd pd_temp1(6,1); 
MatrixXd pd_temp2(6,1);
MatrixXd pd_temp3(6,1); 
MatrixXd pd_temp4(6,1);
MatrixXd pd_temp5(6,1);
MatrixXd pd_temp6(6,1);
MatrixXd pd_temp7(6,1);    
MatrixXd pd_prev(6,1);
MatrixXd pd_offset(6,1);
MatrixXd stepsize(6,1);
MatrixXd dpd(6,1);      //calculated desired cartesian velocity for each step
MatrixXd ddpd(6,1);     //calculated desired cartesian acceleration for each step
MatrixXd qd(7,1);       //calculated desired joint angle for each step
MatrixXd dqd(7,1);      //calculated desired joint angular velocity for each step
MatrixXd ddqd(7,1);     //calculated desired joint acceleration for each step
MatrixXd ddqd_prev(7,1);     //calculated desired joint acceleration for each step
MatrixXd dqd_prev(7,1);
MatrixXd current_q(7,1);
MatrixXd prev_q(7,1);
MatrixXd current_dq(7,1);
MatrixXd prev_dq(7,1);
MatrixXd current_ddq(7,1);
MatrixXd I6(6,6);
double lamda, lamda0, singular_wt, singular_w;
MatrixXd Manipulability(6,6);
MatrixXd jactest(6,7);
MatrixXd dqdprev(7,1);
MatrixXd dqdupt(7,1);
MatrixXd qdprev(7,1);
MatrixXd qdupt(7,1);

MatrixXd drtorque(1,1);

double totaldrivingtorque=0;
double taunorm=0;
double sumtaunorm=0;
double velocitynorm=0;
double accelerationnorm=0;
double errornorm=0;
double maxerrornorm=0;
int trajec_option=0;
int firstrun=0;
double freq=0.001;
double timecounting=0;
double t0=0;
//Moving time 
double T = 5;  //[sec]
double p1[6], p2[6], p3[6], p4[6], pinit[6], pend[6], pdesired[6], dqdesired[7], dqdesired_prev[7], command_tau[7];
unsigned int runcount=0;
unsigned int savedata=0;
double eta1, eta2;
double Kb, damp_factor;


//**********************************************
//Trajectory parameters
double t_total;         //total trajectory time
double t_acc;           //time of acceleration and deceleration process
double N_portion;       //proportion of non-acceleration time N>=2
double linear_disp;     //Linear displacement
double angular_disp;    //angular displacement 
int plan_step;          //Number of planning step
int count_step;         //counting step
//Trapezoid velocity curve params - divided into 7 segments
//V1(t)=linear_a1*t^2+linear_b1*t+linear_c1
//V2(t)=linear_a2*t+linear_b2
//V3(t)=linear_a3*t^2+linear_b3*t+linear_c3
//V4(t)=vmax
//V5(t)=linear_a5*t^2+linear_b5*t+linear_c5
//V6(t)=linear_a6*t+linear_b6
//V7(t)=linear_a7*t^2+linear_b7*t+linear_c7

double linear_V1, linear_V2, linear_V3, linear_V4, linear_V5, linear_V6, linear_V7; 
double linear_a1, linear_a2, linear_a3, linear_a4, linear_a5, linear_a6, linear_a7;
double linear_b1, linear_b2, linear_b3, linear_b4, linear_b5, linear_b6, linear_b7;
double linear_c1, linear_c2, linear_c3, linear_c4, linear_c5, linear_c6, linear_c7;
double linear_vmax;

double angular_V1, angular_V2, angular_V3, angular_V4, angular_V5, angular_V6, angular_V7; 
double angular_a1, angular_a2, angular_a3, angular_a4, angular_a5, angular_a6, angular_a7;
double angular_b1, angular_b2, angular_b3, angular_b4, angular_b5, angular_b6, angular_b7;
double angular_c1, angular_c2, angular_c3, angular_c4, angular_c5, angular_c6, angular_c7;
double angular_vmax;
double linear_dir_vec[3];
double angular_dir_vec[3];

//*************************************SUB PROGRAMS DEFINITION*****************************************
MatrixXd rotx(double angle);    //3x3 rotx about angle
MatrixXd roty(double angle);    //3x3 roty about angle
MatrixXd rotz(double angle);    //3x3 rotz about angle
void delay_ms(int count);
int sgn(double x);
void hd_callback_trans(const downscale4::trans::ConstPtr& msg);         //callback transformation matrix from master device
void hd_callback_buttons(const downscale4::buttons::ConstPtr& msga);    //callback buttons from master device
void hd_callback_keyboards(const downscale4::charmsg::ConstPtr& msga);  //callback from keyboard node
void msgCallbackP(const downscale4::seven::ConstPtr& msg);              //callback joint angle from Gazebo
void msgCallbackT(const downscale4::seven::ConstPtr& msg);              //callback joint torque from Gazebo
void msgCallbackv(const downscale4::seven::ConstPtr& msg);              //callback joint angular velocity from Gazebo
void joint_publish(double theta[7]);                                    //joint angle publishing to Gazebo
void jointt_publish(double tau[7]);                                     //joint torque publishing to Gazebo
void InvK7_1(double pdata[6], double si);                               //inverse kinematics function
void InvK7_0(const Eigen::MatrixXd &data, double si);
MatrixXd forwd7(double qdata[7]);                                       //forward kinematics function
MatrixXd Mass_Matrix(double q[7]);                                      //inertia matrix computation
MatrixXd Gravity_Matrix(double q[7]);                                   //gravity compensate matrix computation
MatrixXd Coriolis_Matrix(double q[7], double dq[7]);                    //coriolis & centrifugal matrix computation
MatrixXd Jacobian_Matrix(double q[7]);                    //jacobian matrix computation
MatrixXd DiffJacobian_Matrix(double q[7], double dq[7]);                //jacobian derivative computation
MatrixXd TorqueOptCase1(double q[7]);                                   //Torque minimization gradient of cost function computation CASE1 - Both Gravity & EE load
MatrixXd TorqueOptCase2(double q[7]);                                   //Torque minimization gradient of cost function computation CASE2 - Gravity only 
MatrixXd TorqueOptCase3(double q[7]);                                   //Torque minimization gradient of cost function computation CASE3 - EE load only
MatrixXd TorqueOptCase4(double q[7]);                                   //Torque minimization gradient of cost function computation proposed
void DataSave(unsigned int count);
void inversedynamics_torqueopt();
void inversedynamics_torqueopt1();
void inversedynamics_torqueopt2();
void cartesianstepcomputing();

//************************************SUB PROGRAMS DESCRIPTION****************************************
void DataSave(unsigned int count)
{
    pData = fopen("MKE.txt","a+");

    fprintf(pData,"%i", count);
    //fprintf(pData,"%f", timer);ㄴ
	/* 
    //Joint errors
    fprintf(pData," %f ", dqdupt(0,0));
    fprintf(pData," %f ", dqdupt(1,0));
    fprintf(pData," %f ", dqdupt(2,0));
    fprintf(pData," %f ", dqdupt(3,0));
    fprintf(pData," %f ", dqdupt(4,0));
    fprintf(pData," %f ", dqdupt(5,0));
    fprintf(pData," %f ", dqdupt(6,0));
    //Joint torque
    fprintf(pData," %f ", Tau_I(0,0));
    fprintf(pData," %f ", Tau_I(1,0));
    fprintf(pData," %f ", Tau_I(2,0));
    fprintf(pData," %f ", Tau_I(3,0));
    fprintf(pData," %f ", Tau_I(4,0));
    fprintf(pData," %f ", Tau_I(5,0));
    fprintf(pData," %f ", Tau_I(6,0));

    //SUM of Norm of dynamic torque
    fprintf(pData," %f ", sumtaunorm);
    */

    //EE Real Position 
    fprintf(pData," %f ", jointpos(0,0));   //x
    fprintf(pData," %f ", jointpos(1,0));   //y
    fprintf(pData," %f ", jointpos(2,0));   //z

    //Joint angle
    fprintf(pData," %f ", current_joint_states[0]);
    fprintf(pData," %f ", current_joint_states[1]);
    fprintf(pData," %f ", current_joint_states[2]);
    fprintf(pData," %f ", current_joint_states[3]);
    fprintf(pData," %f ", current_joint_states[4]);
    fprintf(pData," %f ", current_joint_states[5]);
    fprintf(pData," %f ", current_joint_states[6]);

    //Joint torque
    fprintf(pData," %f ", Tau_I(0,0));
    fprintf(pData," %f ", Tau_I(1,0));
    fprintf(pData," %f ", Tau_I(2,0));
    fprintf(pData," %f ", Tau_I(3,0));
    fprintf(pData," %f ", Tau_I(4,0));
    fprintf(pData," %f ", Tau_I(5,0));
    fprintf(pData," %f ", Tau_I(6,0));

    //Norm of dynamic torque
    fprintf(pData," %f ", taunorm); 

    //SUM of Norm of dynamic torque
    fprintf(pData," %f ", sumtaunorm); 

    //SUM TORQUE DURING TRAJECTORY (integral)
    fprintf(pData," %f ", drtorque(0,0));
    /*    
	//fprintf(pData," %f ", SI_FIX);
    //EE Desired Position 
    fprintf(pData," %f ", pd(0,0));   //x
    fprintf(pData," %f ", pd(1,0));   //y
    fprintf(pData," %f ", pd(2,0));   //z

    //EE Real Position 
    fprintf(pData," %f ", jointpos(0,0));   //x
    fprintf(pData," %f ", jointpos(1,0));   //y
    fprintf(pData," %f ", jointpos(2,0));   //z

    //EE xyz tracking error 
    fprintf(pData," %f ", pderror(0,0));   //x
    fprintf(pData," %f ", pderror(1,0));   //y
    fprintf(pData," %f ", pderror(2,0));   //z


    //Joint angle
    fprintf(pData," %f ", current_joint_states[0]);
    fprintf(pData," %f ", current_joint_states[1]);
    fprintf(pData," %f ", current_joint_states[2]);
    fprintf(pData," %f ", current_joint_states[3]);
    fprintf(pData," %f ", current_joint_states[4]);
    fprintf(pData," %f ", current_joint_states[5]);
    fprintf(pData," %f ", current_joint_states[6]);

    //Joint torque
    fprintf(pData," %f ", Tau_I(0,0));
    fprintf(pData," %f ", Tau_I(1,0));
    fprintf(pData," %f ", Tau_I(2,0));
    fprintf(pData," %f ", Tau_I(3,0));
    fprintf(pData," %f ", Tau_I(4,0));
    fprintf(pData," %f ", Tau_I(5,0));
    fprintf(pData," %f ", Tau_I(6,0));

    //Norm of dynamic torque
    fprintf(pData," %f ", taunorm); 

    //SUM of Norm of dynamic torque
    fprintf(pData," %f ", sumtaunorm); 

    //SUM TORQUE DURING TRAJECTORY (integral)
    fprintf(pData," %f ", drtorque(0,0));

    //EE xyz maximum tracking error 
    fprintf(pData," %f ", maxperror(0,0));   //x
    fprintf(pData," %f ", maxperror(1,0));   //y
    fprintf(pData," %f ", maxperror(2,0));   //z

    //xyz tracking error norm
    fprintf(pData," %f ", errornorm);   //x
    fprintf(pData," %f ", maxerrornorm);   //y

    //Velocity norm acc norm 
    fprintf(pData," %f ", velocitynorm);   //x
    fprintf(pData," %f ", accelerationnorm);   //y
    */ 
    /* 
    fprintf(pData," %f ", Tau_I(0,0));
    fprintf(pData," %f ", Tau_I(1,0));
    fprintf(pData," %f ", Tau_I(2,0));
    fprintf(pData," %f ", Tau_I(3,0));
    fprintf(pData," %f ", Tau_I(4,0));
    fprintf(pData," %f ", Tau_I(5,0));
    fprintf(pData," %f ", Tau_I(6,0));
    
    fprintf(pData," %f ", ddqd(0,0));
    fprintf(pData," %f ", ddqd(1,0));
    fprintf(pData," %f ", ddqd(2,0));
    fprintf(pData," %f ", ddqd(3,0));
    fprintf(pData," %f ", ddqd(4,0));
    fprintf(pData," %f ", ddqd(5,0));
    fprintf(pData," %f ", ddqd(6,0));

    fprintf(pData," %f ", current_joint_states[0]);
    fprintf(pData," %f ", current_joint_states[1]);
    fprintf(pData," %f ", current_joint_states[2]);
    fprintf(pData," %f ", current_joint_states[3]);
    fprintf(pData," %f ", current_joint_states[4]);
    fprintf(pData," %f ", current_joint_states[5]);
    fprintf(pData," %f ", current_joint_states[6]);

    fprintf(pData," %f ", drtorque(0,0));
    */
    
    
    //fprintf(pData," %f ", z_c);
	/*
    fprintf(pData," %f ", x_c);
    fprintf(pData," %f ", y_c);
    fprintf(pData," %f ", z_c);

    fprintf(pData," %f ", posX); //desired x
    fprintf(pData," %f ", posY);//desired y
    fprintf(pData," %f ", posZ);//desired z

    fprintf(pData, "%f ", force_X);
    fprintf(pData," %f ", force_Y);
    fprintf(pData," %f ", force_Z);
	*/
    fprintf(pData,"\n");

    fclose(pData); 

} 
//***************************************Delay ms function********************************************
void delay_ms(int count)
{
	int i,j;
	for (i=0; i<count; i++)
	{
		for (j=0; j<1000; j++)
		{
		}
	}
}

//****************************************sign function***********************************************

int sgn(double x)
{
	if (x > 0) return 1;
	if (x < 0) return -1;
	if (x == 0) return 0;
}

//*****************************Call Back Trans matrix from Master device*******************************
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

		hd_rotation_matrix[0] = -Output_data[2];
		hd_rotation_matrix[1] = -Output_data[0];
		hd_rotation_matrix[2] = Output_data[1];
		hd_rotation_matrix[3] = -Output_data[6];
		hd_rotation_matrix[4] = -Output_data[4];
		hd_rotation_matrix[5] = Output_data[5];
		hd_rotation_matrix[6] = -Output_data[10];
		hd_rotation_matrix[7] = -Output_data[8];
		hd_rotation_matrix[8] = Output_data[9];

		hd_current_xyz[0] = Output_data[12];
		hd_current_xyz[1] = Output_data[13];
		hd_current_xyz[2] = Output_data[14];
}

//********************************Call Back Buttons from Master device*******************************
void hd_callback_buttons(const downscale4::buttons::ConstPtr& msga) {

	Buttons[0] = msga->a;
	Buttons[1] = msga->b;
	Inkwell    = msga->c;

	//std::cout << std::string(80, '-') << std::endl;
}

//**************************************Call Back keyboard********************************************
void hd_callback_keyboards(const downscale4::charmsg::ConstPtr& msga) {

	ReadKey = msga->a;
	//printf(" %c \n",ReadKey);

}
//**********************************Call Back Joint Position******************************************
void msgCallbackP(const downscale4::seven::ConstPtr& msg)
{

	current_joint_states[0] = msg->a;
	current_joint_states[1] = msg->b;
	current_joint_states[2] = msg->c;
	current_joint_states[3] = msg->d;
	current_joint_states[4] = msg->e;
	current_joint_states[5] = msg->f;
	current_joint_states[6] = msg->g;

}

//***********************************Call Back Joint Torque*******************************************
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

//*********************************Call Back Joint Velocities*****************************************
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

//**********************************Joint position publisher******************************************
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

//************************************Joint torque publisher*******************************************
void jointt_publish(double tau[7])
{
	downscale4::seven msgp;
	msgp.a = tau[0];
	msgp.b = tau[1];
	msgp.c = tau[2];
	msgp.d = tau[3];
	msgp.e = tau[4];
	msgp.f = tau[5];
	msgp.g = tau[6];

	pub_jointt.publish(msgp);

}

//*******************************Inverse Kinematics***************************************************
void InvK7_1(double pdata[6], double si)
{
    /*
		description
		inverse kinemaitcs
		input -> cartesian pose 6x1 vector (x,y,z,alpha,beta,gamma); primary arm angle si
		output: joint angles 7x1 vector (q[0]->q[6])
	*/
	double q1dot, q2dot;
	double norm_L_SW;
	int GC2, GC4, GC6;
	//Given homogeneous transformation matrix
    MatrixXd Tinput(4,4);
    MatrixXd Rinput(3,3);
    MatrixXd T_rot_matrix(3,3);
    MatrixXd T_angvel_matrix(3,3);
    MatrixXd ROT_pos_d(3,1);
    MatrixXd ROT_pos_d_2(3,1);
    
    double alpha, beta, gamma;

    //Assign cartesian pose into homogeneous transformation matrix Tinput
    alpha = pdata[3];
    beta = pdata[4];
    gamma = pdata[5];

    Rinput = rotz(gamma)*roty(beta)*rotx(alpha);

    Tinput(0,0)=Rinput(0,0);
    Tinput(1,0)=Rinput(1,0);
    Tinput(2,0)=Rinput(2,0);
    Tinput(3,0)=0;

    Tinput(0,1)=Rinput(0,1);
    Tinput(1,1)=Rinput(1,1);
    Tinput(2,1)=Rinput(2,1);
    Tinput(3,1)=0;

    Tinput(0,2)=Rinput(0,2);
    Tinput(1,2)=Rinput(1,2);
    Tinput(2,2)=Rinput(2,2);
    Tinput(3,2)=0;

    Tinput(0,3)=pdata[0];
    Tinput(1,3)=pdata[1];
    Tinput(2,3)=pdata[2];
    Tinput(3,3)=1;

    //Computation
	DZ << 	0, 0, 1;
	D1 << 	0, 0, d1;
	I = I.setIdentity(3,3);

	//Compute wrist position relating to base from the given EE position end orientation
	W = Tinput.topRightCorner(3,1)-d7*(Tinput.topLeftCorner(3,3)*DZ);	//P(1:3,4)
	//Compute wrist position relating to shoulder
	L_SW = W - D1;
	norm_L_SW = L_SW.norm();
	//Elbow angle q4 in radian
	q[3] = acos((pow(norm_L_SW,2) - pow(d3,2) - pow(d5,2))/(2*d3*d5));
	if (q[3]>=0) GC4 = 1;
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
	XW = R34.transpose()*XS.transpose()*Tinput.topLeftCorner(3,3);
	YW = R34.transpose()*YS.transpose()*Tinput.topLeftCorner(3,3);
	ZW = R34.transpose()*ZS.transpose()*Tinput.topLeftCorner(3,3);

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

void InvK7_0(const Eigen::MatrixXd &data, double si)
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
	if (q[3]>=0) GC4 = 1;
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
//***********************************Forward Kinematics***********************************************
/* 
MatrixXd forwd7(double qdata[7])
{
	MatrixXd pforwk7(3,1);			//computed EE destination postion
	double th1, th2, th3, th4, th5, th6, th7;

	//Update theta from msgCallbackP function
	th1 = qdata[0];
	th2 = qdata[1];
	th3 = qdata[2];
	th4 = qdata[3];
	th5 = qdata[4];
	th6 = qdata[5];
	th7 = qdata[6];

	pforwk7(0,0) = d3 * cos(th1)*sin(th2) - d7 * (cos(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))) - d5 * (sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2));

	pforwk7(1,0) = d5 * (sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) + d7 * (cos(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) + d3 * sin(th1)*sin(th2);

	pforwk7(2,0) = d1 - d7 * (sin(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) - cos(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))) + d5 * (cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4)) + d3 * cos(th2);

	return pforwk7;
}
*/
MatrixXd forwd7(double qdata[7]) //// input: joint angles  output: position + rotation  need 
{
	/*
		description
		forward kinemaitcs
		input -> joint angles
		output: position(x,y,z) , eulerangles(alpha,beta,gamma) -> 6x1 vector (x,y,z,alpha,beta,gamma)
	*/
	MatrixXd pforwk7(6,1);			//computed EE destination postion	
	MatrixXd T01(4,4);
    MatrixXd T12(4,4);
    MatrixXd T23(4,4);
    MatrixXd T34(4,4);
    MatrixXd T45(4,4);
    MatrixXd T56(4,4);
    MatrixXd T67(4,4);
    MatrixXd T7E(4,4);
    MatrixXd T0E(4,4);

    T01 << 	cos(qdata[0]), 	0, 	-sin(qdata[0]), 	0,
                sin(qdata[0]), 	0, 	cos(qdata[0]), 	0,
                    0, 		-1,	 0			, d1,
                    0,		0,	0			,	1; 
                    
    T12 << 	cos(qdata[1]), 0, 	sin(qdata[1]), 0,	
                sin(qdata[1]), 0,     -cos(qdata[1]),0, 	
                    0,		 1,		 	0, 0,
                    0,		0,		0	,	1;  

    T23 << 	cos(qdata[2]), 	0, 	-sin(qdata[2]), 0,	
                sin(qdata[2]), 	0, 	cos(qdata[2]),0, 	
                    0, 		-1, 0,		d3,  
                    0,		0,		0	,	1;

    T34 << 	cos(qdata[3]), 0, 	sin(qdata[3]), 	0,
                sin(qdata[3]), 0,     -cos(qdata[3]), 	0,
                    0,		 1,		 	0,0 ,
                    0,		0,		0	,	1;

    T45 << 	cos(qdata[4]), 	0, 	-sin(qdata[4]), 0,	
                sin(qdata[4]), 	0, 	cos(qdata[4]), 0,  	
                    0, 		-1, 	0,	d5,
                    0,		0,		0	,	1;

    T56 << 	cos(qdata[5]), 0, 	sin(qdata[5]), 	0,
                sin(qdata[5]), 0,     -cos(qdata[5]),0, 	
                    0,		 1,		 	0, 0,
                    0,		0,		0	,	1;

    T67 << 	cos(qdata[6]), -sin(qdata[6]), 	0, 	0,
                sin(qdata[6]), cos(qdata[6]), 	0,	0, 	
                    0, 		0, 		1, 	d7,
                        0,		0,		0	,	1;             

    T7E << 	1, 	0, 	0, 0,	
            0, 	1, 	0, 	0,
                0, 0, 	1, 0,
                0,	0,	0,	1; 

    T0E=T01*T12*T23*T34*T45*T56*T67*T7E;

	pforwk7(0,0) = T0E(0,3);
	pforwk7(1,0) = T0E(1,3);
	pforwk7(2,0) = T0E(2,3);

    // euler calculator
    //R = rotz(gamma)*roty(beta)*rotx(alpha);
    //pforwk7(4,0) <- Beta
    pforwk7(4,0) =atan2(-T0E(2,0),sqrt(pow(T0E(0,0),2)+pow(T0E(1,0),2))); // beta for calculating euler angle in rotation matrix
    if(pforwk7(4,0)==pi/2){
            pforwk7(5,0)=0;                             //pforwk7(5,0) <- Gamma
            pforwk7(3,0)=atan2(T0E(0,1),T0E(1,1));      //pforwk7(3,0) <- Alpha
        }

        else if(pforwk7(4,0)==-pi/2){
            pforwk7(5,0)=0;                             //pforwk7(5,0) <- Gamma
            pforwk7(3,0)=-atan2(T0E(0,1),T0E(1,1));     //pforwk7(3,0) <- Alpha
        }
    else{
            pforwk7(5,0)=atan2(T0E(1,0)/cos(pforwk7(4,0)),T0E(0,0)/cos(pforwk7(4,0))); //pforwk7(5,0) <- Gamma
            pforwk7(3,0)=atan2(T0E(2,1)/cos(pforwk7(4,0)),T0E(2,2)/cos(pforwk7(4,0))); //pforwk7(3,0) <- Alpha
        }

    //printf("frfunc: %.2f %.2f %.2f %.2f %.2f %.2f  \n", robot_current_xyz[0], robot_current_xyz[1], robot_current_xyz[2], current_alpha, current_beta, current_gamma);

    return 	pforwk7;
	
}
//************************************Rotation matrices***********************************************
MatrixXd rotx(double angle)
{
	MatrixXd data(3,3);
	data(0,0) = 1;
	data(0,1) = 0;
	data(0,2) = 0;
	data(1,0) = 0;
	data(1,1) = cos(angle);
	data(1,2) = -sin(angle);
	data(2,0) = 0;
	data(2,1) = sin(angle);
	data(2,2) = cos(angle);
	return data;
}

MatrixXd roty(double angle)
{
	MatrixXd data(3,3);
	data(0,0) = cos(angle);
	data(0,1) = 0;
	data(0,2) = sin(angle);
	data(1,0) = 0;
	data(1,1) = 1;
	data(1,2) = 0;
	data(2,0) = -sin(angle);
	data(2,1) = 0;
	data(2,2) = cos(angle);
	return data;
}

MatrixXd rotz(double angle)
{
	MatrixXd data(3,3);
	data(0,0) = cos(angle);
	data(0,1) = -sin(angle);
	data(0,2) = 0;
	data(1,0) = sin(angle);
	data(1,1) = cos(angle);
	data(1,2) = 0;
	data(2,0) = 0;
	data(2,1) = 0;
	data(2,2) = 1;
	return data;
}

//*****************************************DYNAMICS PARAMS********************************************
//******************************************Mass matrix***********************************************
MatrixXd Mass_Matrix(double q[7])
{
    /*
		description
		Inertia Matrix 
		input -> current joint angles 7x1 vector
		output: inertia matrix 7x7
	*/
    MatrixXd Mass_Comp(7,7);
    
    Mass_Comp(0,0)=(ia1yy + ia2zz + im1zz*pow(kr1,2) + (ia2xx - ia2zz + ia3zz + 2*L2*LL2*m2 + pow(LL2,2)*m2 + pow(L2,2)*(m2 + m3 + m4 + m5 + m6 + m7))*pow(sin(q[1]),2) + 
							((ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*sin(q[2])*
							(sin(q[2]) - pow(cos(q[1]),2)*sin(q[2]) + pow(sin(q[1]),2)*sin(q[2])))/2. + (ia3yy + ia4zz)*L2*(2*cos(q[3])*pow(sin(q[1]),2) + cos(q[2])*sin(2*q[1])*sin(q[3])) + 
							((ia5yy + ia6zz)*(3 + pow(cos(q[3]),2) + (pow(cos(q[1]),2)*(2 - 2*cos(2*q[2]) + cos(2*(q[2] - q[3])) + 6*cos(2*q[3]) + cos(2*(q[2] + q[3]))))/2. - 
								pow(sin(q[1]),2) - 3*pow(cos(q[3]),2)*pow(sin(q[1]),2) - pow(sin(q[2]),2) + pow(cos(q[3]),2)*pow(sin(q[2]),2) - pow(sin(q[1]),2)*pow(sin(q[2]),2) + 
								pow(cos(q[3]),2)*pow(sin(q[1]),2)*pow(sin(q[2]),2) - 8*cos(q[2])*cos(q[3])*sin(2*q[1])*sin(q[3]) - pow(sin(q[3]),2) - 
								(-3 + cos(2*q[1]))*pow(cos(q[2]),2)*pow(sin(q[3]),2) + 3*pow(sin(q[1]),2)*pow(sin(q[3]),2) - pow(sin(q[2]),2)*pow(sin(q[3]),2) - 
								pow(sin(q[1]),2)*pow(sin(q[2]),2)*pow(sin(q[3]),2)))/8. + 
							(ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*
							(pow(cos(q[2]),2)*pow(cos(q[3]),2)*pow(sin(q[1]),2) + pow(cos(q[1]),2)*pow(sin(q[3]),2) + cos(q[1])*cos(q[2])*sin(q[1])*sin(2*q[3])) + 
							(ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
							(pow(cos(q[4]),2)*pow(sin(q[1]),2)*pow(sin(q[2]),2) + 2*cos(q[1])*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + 
							pow(cos(q[1]),2)*pow(sin(q[3]),2)*pow(sin(q[4]),2) - (pow(cos(q[2]),2)*pow(cos(q[3]),2)*sin(q[1])*
								(pow(cos(q[4]),2)*sin(q[1]) - sin(q[1])*(1 + pow(sin(q[4]),2))))/2. + 
							(cos(q[2])*cos(q[3])*(8*cos(q[1])*sin(q[1])*sin(q[3])*pow(sin(q[4]),2) + 4*pow(sin(q[1]),2)*sin(q[2])*sin(2*q[4])))/4.) + 
							(ia5xx - ia5zz + ia6zz)*(pow(cos(q[2]),2)*pow(cos(q[3]),2)*pow(cos(q[4]),2)*pow(sin(q[1]),2) + pow(cos(q[1]),2)*pow(cos(q[4]),2)*pow(sin(q[3]),2) - 
							2*cos(q[1])*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + pow(sin(q[1]),2)*pow(sin(q[2]),2)*pow(sin(q[4]),2) + 
							(cos(q[2])*(4*cos(q[1])*cos(q[3])*pow(cos(q[4]),2)*sin(q[1])*sin(q[3]) - 2*cos(q[3])*pow(sin(q[1]),2)*sin(q[2])*sin(2*q[4])))/2.) + 
							((LL6*m6 + L6*(m6 + m7))*(cos(q[5])*(10*L4 + 8*cos(q[3])*(L2 + L2*pow(sin(q[1]),2)) + 
									L4*pow(cos(q[3]),2)*(-3 + 5*pow(sin(q[1]),2) + cos(2*q[2])*(1 + pow(sin(q[1]),2))) + 2*L4*pow(sin(q[2]),2) + 2*L4*pow(sin(q[3]),2) + 
									2*L4*pow(sin(q[2]),2)*pow(sin(q[3]),2) + L4*pow(cos(q[2]),2)*
									(-2 + (-3 + cos(2*q[3]))*pow(sin(q[1]),2) + 2*pow(cos(q[3]),2)*(1 + pow(sin(q[1]),2)) - 2*pow(sin(q[3]),2)) + 
									pow(cos(q[1]),2)*(-2*L4 - 8*L2*cos(q[3]) - L4*(5 + cos(2*q[2]))*pow(cos(q[3]),2) + L4*(-3 + cos(2*q[3]))*pow(sin(q[2]),2) + 6*L4*pow(sin(q[3]),2) + 
									4*L4*pow(cos(q[2]),2)*pow(sin(q[3]),2)) + 2*pow(sin(q[1]),2)*(L4 - 3*L4*pow(sin(q[3]),2) + L4*pow(sin(q[2]),2)*(1 + pow(sin(q[3]),2))) + 
									8*cos(q[2])*(L2*sin(2*q[1])*sin(q[3]) + L4*sin(2*q[1])*sin(2*q[3]))) + 
								(L4*pow(cos(q[2]),2)*(-3*cos(q[4])*sin(2*q[3]) + cos(2*q[1])*cos(q[4])*sin(2*q[3])) + 
									2*(-4*L2*cos(q[4])*sin(q[3]) + L4*cos(q[4])*sin(2*q[3]) + L4*cos(q[4])*pow(sin(q[2]),2)*sin(2*q[3]) - 
									pow(sin(q[1]),2)*(-(L4*cos(q[4])*pow(sin(q[2]),2)*sin(2*q[3])) + cos(q[4])*(4*L2*sin(q[3]) + 3*L4*sin(2*q[3]))) - 4*L2*sin(2*q[1])*sin(q[2])*sin(q[4]) - 
									4*L4*cos(q[3])*sin(2*q[1])*sin(q[2])*sin(q[4])) + pow(cos(q[1]),2)*
									(8*L2*cos(q[4])*sin(q[3]) + 5*L4*cos(q[4])*sin(2*q[3]) + 2*L4*pow(cos(q[2]),2)*cos(q[4])*sin(2*q[3]) + L4*cos(2*q[2])*cos(q[4])*sin(2*q[3]) - 
									8*L4*cos(q[2])*sin(q[2])*sin(q[3])*sin(q[4])) + 8*cos(q[2])*
									(cos(q[4])*(L2*cos(q[3])*sin(2*q[1]) + L4*pow(cos(q[3]),2)*sin(2*q[1]) - L4*sin(2*q[1])*pow(sin(q[3]),2)) + L4*sin(q[2])*sin(q[3])*sin(q[4]) + 
									L4*pow(sin(q[1]),2)*sin(q[2])*sin(q[3])*sin(q[4])))*sin(q[5])))/8. + 
							(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
							(pow(cos(q[5]),2)*pow(sin(q[1]),2)*pow(sin(q[2]),2)*pow(sin(q[4]),2) + pow(cos(q[1]),2)*pow(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5]),2) + 
							pow(cos(q[2]),2)*(pow(cos(q[3]),2)*pow(cos(q[4]),2)*pow(cos(q[5]),2)*pow(sin(q[1]),2) - 
								2*cos(q[3])*cos(q[4])*cos(q[5])*pow(sin(q[1]),2)*sin(q[3])*sin(q[5]) + pow(sin(q[1]),2)*pow(sin(q[3]),2)*pow(sin(q[5]),2)) - 
							(cos(q[2])*(cos(q[3])*(-4*cos(q[1])*pow(cos(q[4]),2)*pow(cos(q[5]),2)*sin(q[1])*sin(q[3]) + 2*pow(cos(q[5]),2)*pow(sin(q[1]),2)*sin(q[2])*sin(2*q[4])) - 
									4*cos(q[1])*pow(cos(q[3]),2)*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[5]) - 4*cos(q[5])*pow(sin(q[1]),2)*sin(q[2])*sin(q[3])*sin(q[4])*sin(q[5]) - 
									2*cos(q[1])*sin(q[3])*(-2*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[3])*sin(q[5]) - 2*cos(q[3])*sin(q[1])*pow(sin(q[5]),2))))/2. - 
							(cos(q[1])*(2*pow(cos(q[5]),2)*sin(q[1])*sin(q[2])*sin(q[3])*sin(2*q[4]) + 2*cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])*sin(2*q[5])))/2.));

    Mass_Comp(0,1)=((ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*cos(q[2])*sin(q[1])*sin(q[2]) - (ia3yy + ia4zz)*L2*cos(q[1])*sin(q[2])*sin(q[3]) + 
                        (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*
                        (-(cos(q[2])*pow(cos(q[3]),2)*sin(q[1])*sin(q[2])) - cos(q[1])*cos(q[3])*sin(q[2])*sin(q[3])) + 
                        ((ia5yy + ia6zz)*(-4*cos(q[2])*(-(pow(cos(q[3]),2)*sin(q[1])*sin(q[2])) - ((-3 + cos(2*q[3]))*sin(q[1])*sin(q[2]))/2.) + 4*cos(q[1])*sin(q[2])*sin(2*q[3])))/8. + 
                        (ia5xx - ia5zz + ia6zz)*(-(cos(q[1])*cos(q[3])*pow(cos(q[4]),2)*sin(q[2])*sin(q[3])) - pow(cos(q[2]),2)*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[4]) + 
                        cos(q[3])*cos(q[4])*sin(q[1])*pow(sin(q[2]),2)*sin(q[4]) + (cos(q[2])*
                            (-2*pow(cos(q[3]),2)*pow(cos(q[4]),2)*sin(q[1])*sin(q[2]) + 2*(-(cos(q[1])*cos(q[4])*sin(q[3])*sin(q[4])) + sin(q[1])*sin(q[2])*pow(sin(q[4]),2))))/2.) + 
                        (ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                        (-(cos(q[3])*cos(q[4])*sin(q[1])*pow(sin(q[2]),2)*sin(q[4])) - cos(q[1])*cos(q[3])*sin(q[2])*sin(q[3])*pow(sin(q[4]),2) + 
                        (cos(q[2])*(4*pow(cos(q[4]),2)*sin(q[1])*sin(q[2]) + (pow(cos(q[3]),2)*sin(q[1])*(4*pow(cos(q[4]),2)*sin(q[2]) + 2*(-3 + cos(2*q[4]))*sin(q[2])))/2. + 
                                4*cos(q[1])*cos(q[4])*sin(q[3])*sin(q[4])))/4. + (pow(cos(q[2]),2)*cos(q[3])*sin(q[1])*sin(2*q[4]))/2.) + 
                        ((LL6*m6 + L6*(m6 + m7))*(cos(q[5])*(4*cos(q[2])*sin(q[1])*(2*L4*sin(q[2]) - 2*L4*pow(cos(q[3]),2)*sin(q[2]) + 2*L4*sin(q[2])*pow(sin(q[3]),2)) - 
                                2*cos(q[1])*(4*L2*sin(q[2])*sin(q[3]) + 4*L4*sin(q[2])*sin(2*q[3]))) + 
                            (8*L4*pow(cos(q[2]),2)*sin(q[1])*sin(q[3])*sin(q[4]) - 4*sin(q[1])*(-(L4*cos(q[4])*sin(2*q[2])*sin(2*q[3])) + 2*L4*pow(sin(q[2]),2)*sin(q[3])*sin(q[4])) + 
                                8*cos(q[1])*(-(L4*pow(cos(q[3]),2)*cos(q[4])*sin(q[2])) + L4*cos(q[4])*sin(q[2])*pow(sin(q[3]),2) - L2*cos(q[2])*sin(q[4]) - 
                                cos(q[3])*(L2*cos(q[4])*sin(q[2]) + L4*cos(q[2])*sin(q[4]))))*sin(q[5])))/8. + 
                        (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                        (cos(q[3])*cos(q[4])*pow(cos(q[5]),2)*sin(q[1])*pow(sin(q[2]),2)*sin(q[4]) - cos(q[5])*sin(q[1])*pow(sin(q[2]),2)*sin(q[3])*sin(q[4])*sin(q[5]) + 
                        pow(cos(q[2]),2)*(-(cos(q[3])*cos(q[4])*pow(cos(q[5]),2)*sin(q[1])*sin(q[4])) + cos(q[5])*sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5])) - 
                        (cos(q[1])*(2*cos(q[3])*pow(cos(q[4]),2)*pow(cos(q[5]),2)*sin(q[2])*sin(q[3]) + 2*pow(cos(q[3]),2)*cos(q[4])*cos(q[5])*sin(q[2])*sin(q[5]) - 
                                2*cos(q[4])*cos(q[5])*sin(q[2])*pow(sin(q[3]),2)*sin(q[5]) - 2*cos(q[3])*sin(q[2])*sin(q[3])*pow(sin(q[5]),2)))/2. - 
                        (cos(q[2])*(2*pow(cos(q[3]),2)*pow(cos(q[4]),2)*pow(cos(q[5]),2)*sin(q[1])*sin(q[2]) + 2*cos(q[1])*cos(q[4])*pow(cos(q[5]),2)*sin(q[3])*sin(q[4]) + 
                                2*sin(q[1])*(-(pow(cos(q[5]),2)*sin(q[2])*pow(sin(q[4]),2)) + sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[5]),2)) + 
                                cos(q[3])*(2*cos(q[1])*cos(q[5])*sin(q[4])*sin(q[5]) - 2*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(2*q[5]))))/2.));

    Mass_Comp(0,2)=((ia3yy + ia4zz)*L2*cos(q[2])*sin(q[1])*sin(q[3]) + ((ia5yy + ia6zz)*
                    (-8*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[3]) + 4*cos(q[1])*(1 + pow(cos(q[3]),2) - pow(sin(q[3]),2))))/8. + 
                    (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(cos(q[1])*pow(sin(q[3]),2) + (cos(q[2])*sin(q[1])*sin(2*q[3]))/2.) + 
                    (ia5xx - ia5zz + ia6zz)*(cos(q[2])*cos(q[3])*pow(cos(q[4]),2)*sin(q[1])*sin(q[3]) + cos(q[1])*pow(cos(q[4]),2)*pow(sin(q[3]),2) - 
                    cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])) + (ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + cos(q[2])*cos(q[3])*sin(q[1])*sin(q[3])*pow(sin(q[4]),2) - 
                    (cos(q[1])*(2*pow(cos(q[4]),2)*(1 + pow(sin(q[3]),2)) - 2*(1 + pow(sin(q[4]),2) - 2*pow(cos(q[3]),2)*pow(sin(q[4]),2) + 
                            pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2)))))/8.) + ((LL6*m6 + L6*(m6 + m7))*
                    (cos(q[5])*(-2*cos(q[1])*(-4*L4 + 4*L4*pow(cos(q[3]),2) - 4*L4*pow(sin(q[3]),2)) + 4*cos(q[2])*sin(q[1])*(2*L2*sin(q[3]) + 2*L4*sin(2*q[3]))) + 
                        (8*cos(q[2])*(L2*cos(q[3]) + L4*cos(2*q[3]))*cos(q[4])*sin(q[1]) + 8*L4*cos(q[1])*cos(q[4])*sin(2*q[3]) + 
                            2*(-4*L2*sin(q[1])*sin(q[2])*sin(q[4]) - 4*L4*cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])))*sin(q[5])))/8. + 
                    (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (-(cos(q[4])*pow(cos(q[5]),2)*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])) - cos(q[3])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5]) - 
                    (cos(q[1])*(-2*pow(cos(q[4]),2)*pow(cos(q[5]),2)*pow(sin(q[3]),2) - 4*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[3])*sin(q[5]) - 
                            2*pow(cos(q[3]),2)*pow(sin(q[5]),2)))/2. - (cos(q[2])*
                        (-2*cos(q[3])*pow(cos(q[4]),2)*pow(cos(q[5]),2)*sin(q[1])*sin(q[3]) - 2*pow(cos(q[3]),2)*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[5]) + 
                            2*cos(q[4])*cos(q[5])*sin(q[1])*pow(sin(q[3]),2)*sin(q[5]) + 2*cos(q[3])*sin(q[1])*sin(q[3])*pow(sin(q[5]),2)))/2.));

    Mass_Comp(0,3)=((ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*sin(q[1])*sin(q[2]) + (ia3yy + ia4zz)*L2*cos(q[3])*sin(q[1])*sin(q[2]) + 
                    (ia5xx - ia5zz + ia6zz)*(-(cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[4])) - cos(q[1])*cos(q[4])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*pow(sin(q[4]),2)) + 
                    (ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[4]) + cos(q[1])*cos(q[4])*sin(q[3])*sin(q[4]) - (sin(q[1])*sin(q[2])*(-2 - 2*pow(cos(q[4]),2) + 2*pow(sin(q[4]),2)))/4.)\
                    + ((LL6*m6 + L6*(m6 + m7))*(cos(q[5])*(16*L4*sin(q[1])*sin(q[2]) + 8*L2*cos(q[3])*sin(q[1])*sin(q[2])) + 
                        (-8*L2*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3]) - 8*L4*cos(q[1])*cos(q[3])*sin(q[4]) + 8*L4*cos(q[2])*sin(q[1])*sin(q[3])*sin(q[4]))*sin(q[5])))/8. + 
                    (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (pow(cos(q[5]),2)*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - (cos(q[1])*(2*cos(q[4])*pow(cos(q[5]),2)*sin(q[3])*sin(q[4]) + 2*cos(q[3])*cos(q[5])*sin(q[4])*sin(q[5])))/
                        2. - (cos(q[2])*(2*cos(q[3])*cos(q[4])*pow(cos(q[5]),2)*sin(q[1])*sin(q[4]) - 2*cos(q[5])*sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5])))/2.));

    Mass_Comp(0,4)=(((ia5yy + ia6zz)*(8*cos(q[1])*cos(q[3]) - 8*cos(q[2])*sin(q[1])*sin(q[3])))/8. + 
                    ((LL6*m6 + L6*(m6 + m7))*(8*cos(q[2])*(L2 + L4*cos(q[3]))*cos(q[4])*sin(q[1]) + 8*L4*cos(q[1])*cos(q[4])*sin(q[3]) + 
                        2*(-4*L4*sin(q[1])*sin(q[2])*sin(q[4]) - 4*L2*cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])))*sin(q[5]))/8. + 
                    (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (-(cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5])) - (cos(q[1])*(-2*cos(q[4])*cos(q[5])*sin(q[3])*sin(q[5]) - 2*cos(q[3])*pow(sin(q[5]),2)))/2. - 
                    (cos(q[2])*(2*sin(q[1])*sin(q[3])*pow(sin(q[5]),2) - cos(q[3])*cos(q[4])*sin(q[1])*sin(2*q[5])))/2.));

    Mass_Comp(0,5)=((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (cos(q[4])*sin(q[1])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) + cos(q[1])*sin(q[3])*sin(q[4])) + 
                    ((LL6*m6 + L6*(m6 + m7))*(cos(q[5])*(8*L4*cos(q[4])*sin(q[1])*sin(q[2]) + 8*L2*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2]) + 8*L4*cos(q[1])*sin(q[3])*sin(q[4]) + 
                            4*cos(q[2])*sin(q[1])*(2*L2*sin(q[4]) + 2*L4*cos(q[3])*sin(q[4]))) - 8*L2*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])))/8.);

    Mass_Comp(0,6)=0;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mass_Comp(1,0)=((-4*cos(q[1])*(2*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*(2*ia4xx - 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + 
                            pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7 - (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*q[4])))*sin(q[2])*sin(q[3]) + 
                    4*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*pow(cos(q[2]),2)*cos(q[3])*sin(q[1])*sin(2*q[4]) + 
                    sin(q[1])*(sin(2*q[2])*(4*ia4yy + 4*ia5zz + ia6yy + ia7yy + 4*pow(L4,2)*m4 + 8*L4*LL4*m4 + 4*pow(LL4,2)*m4 + 4*pow(L4,2)*m5 + 4*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + 4*pow(L4,2)*m7 + L6*m7 + 
                            pow(L6,2)*m7 + 6*L6*LL6*m6*pow(cos(q[4]),2) + pow(cos(q[3]),2)*(-8*L4*LL4*m4 + (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*pow(cos(q[4]),2) + L6*LL6*m6*cos(2*q[4])) + 
                            ia6yy*pow(sin(q[3]),2) + ia7yy*pow(sin(q[3]),2) + L6*m6*pow(sin(q[3]),2) + pow(L6,2)*m6*pow(sin(q[3]),2) + LL6*m6*pow(sin(q[3]),2) + 2*L6*LL6*m6*pow(sin(q[3]),2) + pow(LL6,2)*m6*pow(sin(q[3]),2) + L6*m7*pow(sin(q[3]),2) + 
                            pow(L6,2)*m7*pow(sin(q[3]),2) - 6*L6*LL6*m6*pow(sin(q[4]),2) + ia6yy*pow(sin(q[3]),2)*pow(sin(q[4]),2) + ia7yy*pow(sin(q[3]),2)*pow(sin(q[4]),2) + L6*m6*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 
                            pow(L6,2)*m6*pow(sin(q[3]),2)*pow(sin(q[4]),2) + LL6*m6*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 2*L6*LL6*m6*pow(sin(q[3]),2)*pow(sin(q[4]),2) + pow(LL6,2)*m6*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 
                            L6*m7*pow(sin(q[3]),2)*pow(sin(q[4]),2) + pow(L6,2)*m7*pow(sin(q[3]),2)*pow(sin(q[4]),2)) - 4*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(q[3])*pow(sin(q[2]),2)*sin(2*q[4])
                        ) + cos(q[2])*((5*ia6yy + 5*ia7yy + 5*L6*m6 + 5*pow(L6,2)*m6 + 5*LL6*m6 - 2*L6*LL6*m6 + 5*pow(LL6,2)*m6 + 5*L6*m7 + 5*pow(L6,2)*m7 + 
                            (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*q[3]))*pow(cos(q[4]),2)*sin(q[1])*sin(q[2]) - 
                        pow(cos(q[3]),2)*(8*ia4xx - 8*ia4zz + 8*ia5zz + 3*ia6yy + 3*ia7yy + 8*pow(L4,2)*m4 + 8*pow(LL4,2)*m4 + 8*pow(L4,2)*m5 + 8*pow(L4,2)*m6 + 3*L6*m6 + 3*pow(L6,2)*m6 + 3*LL6*m6 + 4*L6*LL6*m6 + 3*pow(LL6,2)*m6 + 8*pow(L4,2)*m7 + 
                            3*L6*m7 + 3*pow(L6,2)*m7 - (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*q[4]))*sin(q[1])*sin(q[2]) - 6*ia6yy*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 
                        6*ia7yy*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 6*L6*m6*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 6*pow(L6,2)*m6*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 6*LL6*m6*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 
                        6*pow(LL6,2)*m6*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 6*L6*m7*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) - 6*pow(L6,2)*m7*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) + 8*L6*LL6*m6*cos(q[1])*sin(q[3])*sin(2*q[4]) + 
                        4*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(q[1])*sin(q[3])*sin(2*q[4])))/8.);

    Mass_Comp(1,1)=((32*ia2yy + 32*ia3zz + 8*ia4xx + 16*ia4yy - 8*ia4zz + 24*ia5zz + 12*ia6yy + 12*ia7yy + 32*im2zz*pow(kr2,2) + 32*pow(L2,2)*m2 + 64*L2*LL2*m2 + 32*pow(LL2,2)*m2 + 32*pow(L2,2)*m3 + 32*pow(L2,2)*m4 + 24*pow(L4,2)*m4 + 48*L4*LL4*m4 + 
                    24*pow(LL4,2)*m4 + 32*pow(L2,2)*m5 + 24*pow(L4,2)*m5 + 32*pow(L2,2)*m6 + 24*pow(L4,2)*m6 + 12*L6*m6 + 12*pow(L6,2)*m6 + 12*LL6*m6 + 24*L6*LL6*m6 + 12*pow(LL6,2)*m6 + 32*pow(L2,2)*m7 + 24*pow(L4,2)*m7 + 12*L6*m7 + 
                    12*pow(L6,2)*m7 + 4*(-2*ia4xx + 4*ia4yy + 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + 
                        2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7)*cos(2*q[2]) - 2*(2*ia4xx - 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + 
                        pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7)*cos(2*(q[2] - q[3])) + 64*L2*L4*m4*cos(q[3]) + 64*L2*LL4*m4*cos(q[3]) + 64*L2*L4*m5*cos(q[3]) + 64*L2*L4*m6*cos(q[3]) + 64*L2*L4*m7*cos(q[3]) + 8*ia4xx*cos(2*q[3]) - 8*ia4zz*cos(2*q[3]) + 
                    8*ia5zz*cos(2*q[3]) + 4*ia6yy*cos(2*q[3]) + 4*ia7yy*cos(2*q[3]) + 8*pow(L4,2)*m4*cos(2*q[3]) + 16*L4*LL4*m4*cos(2*q[3]) + 8*pow(LL4,2)*m4*cos(2*q[3]) + 8*pow(L4,2)*m5*cos(2*q[3]) + 8*pow(L4,2)*m6*cos(2*q[3]) + 4*L6*m6*cos(2*q[3]) + 
                    4*pow(L6,2)*m6*cos(2*q[3]) + 4*LL6*m6*cos(2*q[3]) + 8*L6*LL6*m6*cos(2*q[3]) + 4*pow(LL6,2)*m6*cos(2*q[3]) + 8*pow(L4,2)*m7*cos(2*q[3]) + 4*L6*m7*cos(2*q[3]) + 4*pow(L6,2)*m7*cos(2*q[3]) - 4*ia4xx*cos(2*(q[2] + q[3])) + 4*ia4zz*cos(2*(q[2] + q[3])) - 
                    4*ia5zz*cos(2*(q[2] + q[3])) - 2*ia6yy*cos(2*(q[2] + q[3])) - 2*ia7yy*cos(2*(q[2] + q[3])) - 4*pow(L4,2)*m4*cos(2*(q[2] + q[3])) - 8*L4*LL4*m4*cos(2*(q[2] + q[3])) - 4*pow(LL4,2)*m4*cos(2*(q[2] + q[3])) - 4*pow(L4,2)*m5*cos(2*(q[2] + q[3])) - 
                    4*pow(L4,2)*m6*cos(2*(q[2] + q[3])) - 2*L6*m6*cos(2*(q[2] + q[3])) - 2*pow(L6,2)*m6*cos(2*(q[2] + q[3])) - 2*LL6*m6*cos(2*(q[2] + q[3])) - 4*L6*LL6*m6*cos(2*(q[2] + q[3])) - 2*pow(LL6,2)*m6*cos(2*(q[2] + q[3])) - 4*pow(L4,2)*m7*cos(2*(q[2] + q[3])) - 
                    2*L6*m7*cos(2*(q[2] + q[3])) - 2*pow(L6,2)*m7*cos(2*(q[2] + q[3])) - 4*ia6yy*cos(2*q[2] - q[3] - 2*q[4]) - 4*ia7yy*cos(2*q[2] - q[3] - 2*q[4]) - 4*L6*m6*cos(2*q[2] - q[3] - 2*q[4]) - 4*pow(L6,2)*m6*cos(2*q[2] - q[3] - 2*q[4]) - 
                    4*LL6*m6*cos(2*q[2] - q[3] - 2*q[4]) - 8*L6*LL6*m6*cos(2*q[2] - q[3] - 2*q[4]) - 4*pow(LL6,2)*m6*cos(2*q[2] - q[3] - 2*q[4]) - 4*L6*m7*cos(2*q[2] - q[3] - 2*q[4]) - 4*pow(L6,2)*m7*cos(2*q[2] - q[3] - 2*q[4]) - 4*ia6yy*cos(2*q[2] + q[3] - 2*q[4]) - 
                    4*ia7yy*cos(2*q[2] + q[3] - 2*q[4]) - 4*L6*m6*cos(2*q[2] + q[3] - 2*q[4]) - 4*pow(L6,2)*m6*cos(2*q[2] + q[3] - 2*q[4]) - 4*LL6*m6*cos(2*q[2] + q[3] - 2*q[4]) - 8*L6*LL6*m6*cos(2*q[2] + q[3] - 2*q[4]) - 4*pow(LL6,2)*m6*cos(2*q[2] + q[3] - 2*q[4]) - 
                    4*L6*m7*cos(2*q[2] + q[3] - 2*q[4]) - 4*pow(L6,2)*m7*cos(2*q[2] + q[3] - 2*q[4]) + 6*ia6yy*cos(2*(q[2] - q[4])) + 6*ia7yy*cos(2*(q[2] - q[4])) + 6*L6*m6*cos(2*(q[2] - q[4])) + 6*pow(L6,2)*m6*cos(2*(q[2] - q[4])) + 6*LL6*m6*cos(2*(q[2] - q[4])) + 
                    12*L6*LL6*m6*cos(2*(q[2] - q[4])) + 6*pow(LL6,2)*m6*cos(2*(q[2] - q[4])) + 6*L6*m7*cos(2*(q[2] - q[4])) + 6*pow(L6,2)*m7*cos(2*(q[2] - q[4])) + ia6yy*cos(2*(q[2] - q[3] - q[4])) + ia7yy*cos(2*(q[2] - q[3] - q[4])) + L6*m6*cos(2*(q[2] - q[3] - q[4])) + 
                    pow(L6,2)*m6*cos(2*(q[2] - q[3] - q[4])) + LL6*m6*cos(2*(q[2] - q[3] - q[4])) + 2*L6*LL6*m6*cos(2*(q[2] - q[3] - q[4])) + pow(LL6,2)*m6*cos(2*(q[2] - q[3] - q[4])) + L6*m7*cos(2*(q[2] - q[3] - q[4])) + pow(L6,2)*m7*cos(2*(q[2] - q[3] - q[4])) - 
                    2*ia6yy*cos(2*(q[3] - q[4])) - 2*ia7yy*cos(2*(q[3] - q[4])) - 2*L6*m6*cos(2*(q[3] - q[4])) - 2*pow(L6,2)*m6*cos(2*(q[3] - q[4])) - 2*LL6*m6*cos(2*(q[3] - q[4])) - 4*L6*LL6*m6*cos(2*(q[3] - q[4])) - 2*pow(LL6,2)*m6*cos(2*(q[3] - q[4])) - 
                    2*L6*m7*cos(2*(q[3] - q[4])) - 2*pow(L6,2)*m7*cos(2*(q[3] - q[4])) + ia6yy*cos(2*(q[2] + q[3] - q[4])) + ia7yy*cos(2*(q[2] + q[3] - q[4])) + L6*m6*cos(2*(q[2] + q[3] - q[4])) + pow(L6,2)*m6*cos(2*(q[2] + q[3] - q[4])) + LL6*m6*cos(2*(q[2] + q[3] - q[4])) + 
                    2*L6*LL6*m6*cos(2*(q[2] + q[3] - q[4])) + pow(LL6,2)*m6*cos(2*(q[2] + q[3] - q[4])) + L6*m7*cos(2*(q[2] + q[3] - q[4])) + pow(L6,2)*m7*cos(2*(q[2] + q[3] - q[4])) + 4*ia6yy*cos(2*q[4]) + 4*ia7yy*cos(2*q[4]) + 4*L6*m6*cos(2*q[4]) + 4*pow(L6,2)*m6*cos(2*q[4]) + 
                    4*LL6*m6*cos(2*q[4]) + 8*L6*LL6*m6*cos(2*q[4]) + 4*pow(LL6,2)*m6*cos(2*q[4]) + 4*L6*m7*cos(2*q[4]) + 4*pow(L6,2)*m7*cos(2*q[4]) + 6*ia6yy*cos(2*(q[2] + q[4])) + 6*ia7yy*cos(2*(q[2] + q[4])) + 6*L6*m6*cos(2*(q[2] + q[4])) + 
                    6*pow(L6,2)*m6*cos(2*(q[2] + q[4])) + 6*LL6*m6*cos(2*(q[2] + q[4])) + 12*L6*LL6*m6*cos(2*(q[2] + q[4])) + 6*pow(LL6,2)*m6*cos(2*(q[2] + q[4])) + 6*L6*m7*cos(2*(q[2] + q[4])) + 6*pow(L6,2)*m7*cos(2*(q[2] + q[4])) + ia6yy*cos(2*(q[2] - q[3] + q[4])) + 
                    ia7yy*cos(2*(q[2] - q[3] + q[4])) + L6*m6*cos(2*(q[2] - q[3] + q[4])) + pow(L6,2)*m6*cos(2*(q[2] - q[3] + q[4])) + LL6*m6*cos(2*(q[2] - q[3] + q[4])) + 2*L6*LL6*m6*cos(2*(q[2] - q[3] + q[4])) + pow(LL6,2)*m6*cos(2*(q[2] - q[3] + q[4])) + 
                    L6*m7*cos(2*(q[2] - q[3] + q[4])) + pow(L6,2)*m7*cos(2*(q[2] - q[3] + q[4])) - 2*ia6yy*cos(2*(q[3] + q[4])) - 2*ia7yy*cos(2*(q[3] + q[4])) - 2*L6*m6*cos(2*(q[3] + q[4])) - 2*pow(L6,2)*m6*cos(2*(q[3] + q[4])) - 2*LL6*m6*cos(2*(q[3] + q[4])) - 
                    4*L6*LL6*m6*cos(2*(q[3] + q[4])) - 2*pow(LL6,2)*m6*cos(2*(q[3] + q[4])) - 2*L6*m7*cos(2*(q[3] + q[4])) - 2*pow(L6,2)*m7*cos(2*(q[3] + q[4])) + ia6yy*cos(2*(q[2] + q[3] + q[4])) + ia7yy*cos(2*(q[2] + q[3] + q[4])) + L6*m6*cos(2*(q[2] + q[3] + q[4])) + 
                    pow(L6,2)*m6*cos(2*(q[2] + q[3] + q[4])) + LL6*m6*cos(2*(q[2] + q[3] + q[4])) + 2*L6*LL6*m6*cos(2*(q[2] + q[3] + q[4])) + pow(LL6,2)*m6*cos(2*(q[2] + q[3] + q[4])) + L6*m7*cos(2*(q[2] + q[3] + q[4])) + pow(L6,2)*m7*cos(2*(q[2] + q[3] + q[4])) + 
                    4*ia6yy*cos(2*q[2] - q[3] + 2*q[4]) + 4*ia7yy*cos(2*q[2] - q[3] + 2*q[4]) + 4*L6*m6*cos(2*q[2] - q[3] + 2*q[4]) + 4*pow(L6,2)*m6*cos(2*q[2] - q[3] + 2*q[4]) + 4*LL6*m6*cos(2*q[2] - q[3] + 2*q[4]) + 8*L6*LL6*m6*cos(2*q[2] - q[3] + 2*q[4]) + 
                    4*pow(LL6,2)*m6*cos(2*q[2] - q[3] + 2*q[4]) + 4*L6*m7*cos(2*q[2] - q[3] + 2*q[4]) + 4*pow(L6,2)*m7*cos(2*q[2] - q[3] + 2*q[4]) + 4*ia6yy*cos(2*q[2] + q[3] + 2*q[4]) + 4*ia7yy*cos(2*q[2] + q[3] + 2*q[4]) + 4*L6*m6*cos(2*q[2] + q[3] + 2*q[4]) + 
                    4*pow(L6,2)*m6*cos(2*q[2] + q[3] + 2*q[4]) + 4*LL6*m6*cos(2*q[2] + q[3] + 2*q[4]) + 8*L6*LL6*m6*cos(2*q[2] + q[3] + 2*q[4]) + 4*pow(LL6,2)*m6*cos(2*q[2] + q[3] + 2*q[4]) + 4*L6*m7*cos(2*q[2] + q[3] + 2*q[4]) + 4*pow(L6,2)*m7*cos(2*q[2] + q[3] + 2*q[4]))/32.);

    Mass_Comp(1,2)=((sin(q[3])*((-4*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7)) - 2*cos(q[3])*(2*ia4xx - 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + 
                        pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7 - (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*q[4])))*sin(q[2]) + 
                    2*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(q[2])*sin(2*q[4])))/4.);

    Mass_Comp(1,3)=((cos(q[2])*(2*ia4yy + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + 
                    LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7 + 2*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*cos(q[3]) + 
                    (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*q[4])) - 
                (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(q[3])*sin(q[2])*sin(2*q[4]))/2.);

    Mass_Comp(1,4)=0;
    Mass_Comp(1,5)=((ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*(cos(q[2])*cos(q[4]) - cos(q[3])*sin(q[2])*sin(q[4])));
    Mass_Comp(1,6)=0;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mass_Comp(2,0)=(im3zz*kr3*cos(q[1]) + L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*cos(q[2])*sin(q[1])*sin(q[3]) + 
                    (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*sin(q[3])*(cos(q[2])*cos(q[3])*sin(q[1]) + cos(q[1])*sin(q[3])) + 
                    ((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*sin(q[4])*
                    (2*cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3]) + (2*cos(q[1])*pow(sin(q[3]),2) + cos(q[2])*sin(q[1])*sin(2*q[3]))*sin(q[4])))/2. + 
                    ((LL6*m6 + L6*(m6 + m7))*(2*cos(q[5])*sin(q[3])*(cos(q[2])*(L2 + 2*L4*cos(q[3]))*sin(q[1]) + 2*L4*cos(q[1])*sin(q[3])) + 
                        2*(cos(q[2])*(L2*cos(q[3]) + L4*cos(2*q[3]))*cos(q[4])*sin(q[1]) + L4*cos(q[1])*cos(q[4])*sin(2*q[3]) - (L2 + L4*cos(q[3]))*sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5])))/2. + 
                    (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5]))*
                    (-(cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])) + cos(q[1])*(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5])) + 
                    cos(q[2])*sin(q[1])*(cos(q[3])*cos(q[4])*cos(q[5]) - sin(q[3])*sin(q[5]))));

    Mass_Comp(2,1)=(-(L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*sin(q[2])*sin(q[3])) - (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*cos(q[3])*
                    sin(q[2])*sin(q[3]) + ((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (-8*sin(q[2])*sin(2*q[3])*pow(sin(q[4]),2) + 8*cos(q[2])*sin(q[3])*sin(2*q[4])))/16. - 
                    (ia6xx - ia6zz + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5]))*
                    (cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) + cos(q[2])*cos(q[5])*sin(q[4]) - sin(q[2])*sin(q[3])*sin(q[5])) - 
                    (LL6*m6 + L6*(m6 + m7))*((L2 + 2*L4*cos(q[3]))*cos(q[5])*sin(q[2])*sin(q[3]) + 
                    (L4*pow(cos(q[3]),2)*cos(q[4])*sin(q[2]) - L4*cos(q[4])*sin(q[2])*pow(sin(q[3]),2) + L2*cos(q[2])*sin(q[4]) + 
                        cos(q[3])*(L2*cos(q[4])*sin(q[2]) + L4*cos(q[2])*sin(q[4])))*sin(q[5])));

    Mass_Comp(2,2)=(im3zz*pow(kr3,2) + (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*pow(sin(q[3]),2) + 
                    (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 
                    (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*pow(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5]),2) + 
                    L4*(LL6*m6 + L6*(m6 + m7))*(2*cos(q[5])*pow(sin(q[3]),2) + cos(q[4])*sin(2*q[3])*sin(q[5])));

    Mass_Comp(2,3)=(((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*sin(q[3])*sin(2*q[4]))/2. - L4*(LL6*m6 + L6*(m6 + m7))*cos(q[3])*sin(q[4])*sin(q[5]) - 
                    (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*cos(q[5])*sin(q[4])*(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5])));

    Mass_Comp(2,4)=(sin(q[5])*(L4*(LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3]) + (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
                    (cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5]))));

    Mass_Comp(2,5)=((ia6yy + ia7yy + im6zz*kr6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7) + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5]))*sin(q[3])*sin(q[4]));
    Mass_Comp(2,6)=0;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mass_Comp(3,0)=(cos(q[1])*((ia6yy + ia7yy + pow(L6,2)*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(q[4])*sin(q[3])*sin(q[4]) + L6*LL6*m6*sin(q[3])*sin(2*q[4]) - 
                    L4*(LL6*m6 + L6*(m6 + m7))*cos(q[3])*sin(q[4])*sin(q[5])) + sin(q[1])*
                    (cos(q[2])*sin(q[4])*((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(q[3])*cos(q[4]) + 
                        L4*(LL6*m6 + L6*(m6 + m7))*sin(q[3])*sin(q[5])) + sin(q[2])*(ia4yy + ia5zz + im4zz*kr4 + pow(L4,2)*m4 + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*m5 + 
                        pow(L4,2)*m6 + pow(L4,2)*m7 + (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(cos(q[4]),2) + 2*L4*L6*m6*cos(q[5]) + 
                        2*L4*LL6*m6*cos(q[5]) + 2*L4*L6*m7*cos(q[5]) + L2*cos(q[3])*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5])) - 
                        L2*(LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3])*sin(q[5]))));

    Mass_Comp(3,1)=(-(sin(q[2])*sin(q[4])*((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(q[3])*cos(q[4]) + 
                        L4*(LL6*m6 + L6*(m6 + m7))*sin(q[3])*sin(q[5]))) + cos(q[2])*(ia4yy + ia5zz + im4zz*kr4 + pow(L4,2)*m4 + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*m5 + 
                    pow(L4,2)*m6 + pow(L4,2)*m7 + (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(cos(q[4]),2) + 2*L4*L6*m6*cos(q[5]) + 
                    2*L4*LL6*m6*cos(q[5]) + 2*L4*L6*m7*cos(q[5]) + L2*cos(q[3])*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5])) - 
                    L2*(LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3])*sin(q[5])));

    Mass_Comp(3,2)=(sin(q[4])*((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(q[4])*sin(q[3]) - L4*(LL6*m6 + L6*(m6 + m7))*cos(q[3])*sin(q[5])));

    Mass_Comp(3,3)=(ia4yy + ia5zz + im4zz*pow(kr4,2) + pow(L4,2)*m4 + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*m5 + pow(L4,2)*m6 + pow(L4,2)*m7 + 
                    (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(cos(q[4]),2) + 2*L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5]));

    Mass_Comp(3,4)=(-(L4*(LL6*m6 + L6*(m6 + m7))*sin(q[4])*sin(q[5])));
    Mass_Comp(3,5)=cos(q[4])*(ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5]));
    Mass_Comp(3,6)=0;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mass_Comp(4,0)=(cos(q[1])*((ia5yy + ia6zz + im5zz*kr5)*cos(q[3]) + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3])*sin(q[5])) - 
                    sin(q[1])*((LL6*m6 + L6*(m6 + m7))*(L4 + L2*cos(q[3]))*sin(q[2])*sin(q[4])*sin(q[5]) + 
                    cos(q[2])*((ia5yy + ia6zz + im5zz*kr5)*sin(q[3]) - (LL6*m6 + L6*(m6 + m7))*(L2 + L4*cos(q[3]))*cos(q[4])*sin(q[5]))));

    Mass_Comp(4,1)=((-(LL6*m6) - L6*(m6 + m7))*cos(q[2])*(L4 + L2*cos(q[3]))*sin(q[4])*sin(q[5]) + 
                sin(q[2])*((ia5yy + ia6zz + im5zz*kr5)*sin(q[3]) - (LL6*m6 + L6*(m6 + m7))*(L2 + L4*cos(q[3]))*cos(q[4])*sin(q[5])));

    Mass_Comp(4,2)=((ia5yy + ia6zz + im5zz*kr5)*cos(q[3]) + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3])*sin(q[5]));
    Mass_Comp(4,3)=(-(L4*(LL6*m6 + L6*(m6 + m7))*sin(q[4])*sin(q[5])));
    Mass_Comp(4,4)=(ia5yy + ia6zz + im5zz*pow(kr5,2));
    Mass_Comp(4,5)=0;
    Mass_Comp(4,6)=0;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mass_Comp(5,0)=(cos(q[4])*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + (LL6*m6 + L6*(m6 + m7))*(L4 + L2*cos(q[3]))*cos(q[5]))*
                    sin(q[1])*sin(q[2]) + cos(q[2])*(L2*(LL6*m6 + L6*(m6 + m7))*cos(q[5]) + 
                    cos(q[3])*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5])))*sin(q[1])*sin(q[4])\
                    + sin(q[3])*(cos(q[1])*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5]))*
                        sin(q[4]) - L2*(LL6*m6 + L6*(m6 + m7))*sin(q[1])*sin(q[2])*sin(q[5])));

    Mass_Comp(5,1)=((-(L2*(LL6*m6 + L6*(m6 + m7))*cos(q[5])) - cos(q[3])*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + 
                        L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5])))*sin(q[2])*sin(q[4]) + 
                    cos(q[2])*(cos(q[4])*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + 
                        (LL6*m6 + L6*(m6 + m7))*(L4 + L2*cos(q[3]))*cos(q[5])) - L2*(LL6*m6 + L6*(m6 + m7))*sin(q[3])*sin(q[5])));

    Mass_Comp(5,2)=((ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5]))*sin(q[3])*sin(q[4]));
    Mass_Comp(5,3)=(cos(q[4])*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(q[5])));
    Mass_Comp(5,4)=0;
    Mass_Comp(5,5)=(ia6yy + ia7yy + im6zz*pow(kr6,2) + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7);
    Mass_Comp(5,6)=0;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------
    Mass_Comp(6,0)=((ia7zz + im7zz*kr7)*(cos(q[1])*(cos(q[3])*cos(q[5]) - cos(q[4])*sin(q[3])*sin(q[5])) + 
                    sin(q[1])*(sin(q[2])*sin(q[4])*sin(q[5]) - cos(q[2])*(cos(q[5])*sin(q[3]) + cos(q[3])*cos(q[4])*sin(q[5])))));
    Mass_Comp(6,1)=((ia7zz + im7zz*kr7)*(cos(q[5])*sin(q[2])*sin(q[3]) + (cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4]))*sin(q[5])));
    Mass_Comp(6,2)=((ia7zz + im7zz*kr7)*(cos(q[3])*cos(q[5]) - cos(q[4])*sin(q[3])*sin(q[5])));
    Mass_Comp(6,3)=((ia7zz + im7zz*kr7)*sin(q[4])*sin(q[5]));
    Mass_Comp(6,4)=((ia7zz + im7zz*kr7)*cos(q[5]));
    Mass_Comp(6,5)=0;
    Mass_Comp(6,6)=(ia7zz + im7zz*pow(kr7,2));

    return Mass_Comp;
}

//***********************************Gravity Compensate matrix****************************************
//ISSAC
MatrixXd Gravity_Matrix(double q[7])
{
    /*
		description
		Gravity Compensate Matrix 
		input -> current joint angles 7x1 vector
		output: Gravity Compensate vector 7x1
	*/
    MatrixXd Grav_Comp(7,1);

    Grav_Comp(0,0)=0;

    Grav_Comp(1,0)=g_acc*(-(sin(q[1])*(L2*m2 + LL2*m2 + L2*m3 + L2*m4 + L2*m5 + L2*m6 + L2*m7 + cos(q[3])*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5])) - 
                    (LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3])*sin(q[5]))) + cos(q[1])*
                ((LL6*m6 + L6*(m6 + m7))*sin(q[2])*sin(q[4])*sin(q[5]) - cos(q[2])*
                    ((LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5]))*sin(q[3]) + (LL6*m6 + L6*(m6 + m7))*cos(q[3])*cos(q[4])*sin(q[5]))));
                
    Grav_Comp(2,0)=g_acc*(sin(q[1])*((LL6*m6 + L6*(m6 + m7))*cos(q[2])*sin(q[4])*sin(q[5]) + 
                    sin(q[2])*((LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5]))*sin(q[3]) + (LL6*m6 + L6*(m6 + m7))*cos(q[3])*cos(q[4])*sin(q[5]))));

    Grav_Comp(3,0)=g_acc*(-(cos(q[1])*((LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5]))*sin(q[3]) + (LL6*m6 + L6*(m6 + m7))*cos(q[3])*cos(q[4])*sin(q[5]))) - 
                cos(q[2])*sin(q[1])*(cos(q[3])*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(q[5])) - (LL6*m6 + L6*(m6 + m7))*cos(q[4])*sin(q[3])*sin(q[5])));

    Grav_Comp(4,0)=g_acc*((LL6*m6 + L6*(m6 + m7))*(cos(q[4])*sin(q[1])*sin(q[2]) + (cos(q[2])*cos(q[3])*sin(q[1]) + cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]));

    Grav_Comp(5,0)=g_acc*((-(LL6*m6) - L6*(m6 + m7))*(-(cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])) + cos(q[1])*(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5])) + 
                    cos(q[2])*sin(q[1])*(cos(q[3])*cos(q[4])*cos(q[5]) - sin(q[3])*sin(q[5]))));

    Grav_Comp(6,0)=0;

    return Grav_Comp;
}
//***********************************Gravity Compensate matrix****************************************
//TIEN
/* 
double gravitycomptorque(int axis, double q[7])
{
	double Gravity_T;
	if (axis == 0)
	{
		//Joint 1 = 0 most cases			
		Gravity_T = 0;
	}	
	if (axis == 1)
	{
		//Joint 2			
		Gravity_T = - g_acc*m[6]*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) - g_acc*m[4]*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) - g_acc*m[5]*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) - g_acc*lg3*m[2]*sin(q[1]) - g_acc*lg4*m[3]*sin(q[1]);
	}
	if (axis == 2)
	{
		//Joint 3 = 0 most cases			
		Gravity_T = g_acc*sin(q[1])*(lg5*m[4]*sin(q[2])*sin(q[3]) + lg6*m[5]*sin(q[2])*sin(q[3]) + d5*m[6]*sin(q[2])*sin(q[3]) + lg7*m[6]*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m[6]*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m[6]*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5]));
	}
	if (axis == 3)
	{
		//Joint 4			
		Gravity_T = - g_acc*m[6]*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - g_acc*lg5*m[4]*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - g_acc*lg6*m[5]*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]));
	}
	if (axis == 4)
	{
		//Joint 5 = 0 most cases			
		Gravity_T = g_acc*lg7*m[6]*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2]));
	}
	if (axis == 5)
	{
		//Joint 6			
		Gravity_T = - g_acc*lg7*m[6]*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])));
	}
	if (axis == 6)
	{
		//Joint 7 = 0 			
		Gravity_T = 0;
	}
	
	printf("Gravity compensate [%d] = %.5f \n", axis+1, Gravity_T);
	return Gravity_T;
	
}
*/

//***********************************Coriolis & Centrifugal matrix****************************************
MatrixXd Coriolis_Matrix(double q[7], double dq[7])
{
    /*
		description
		Coriolis & Centrifugal Matrix 
		input -> current joint angles 7x1 vector, joint anglular velocity 7x1 vector
		output: Coriolis & Centrifugal Matrix 7x1
	*/
    MatrixXd Coriolis_Comp(7,1);
    Coriolis_Comp(0,0)= 0; // not yet

    Coriolis_Comp(1,0)=((32*(ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(-2*dq[0]*cos(q[1])*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])) + dq[2]*(dq[0]*sin(q[1] - 2*q[2]) - 2*dq[3]*sin(q[2]) - 2*dq[1]*sin(2*q[2]) + dq[0]*sin(q[1] + 2*q[2]))) + 
        64*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*(-2*L2*cos(q[3])*(dq[2]*dq[3]*sin(q[2]) + dq[0]*cos(q[1])*(dq[0]*sin(q[1]) + dq[3]*sin(q[2]))) - L2*(2*dq[1]*dq[3] + (pow(dq[2],2) + pow(dq[3],2) + 2*dq[0]*dq[2]*cos(q[1]) + pow(dq[0],2)*cos(2*q[1]))*cos(q[2]))*sin(q[3])) + 
        64*(ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(-(dq[0]*(dq[2] + dq[0]*cos(q[1]))*pow(cos(q[2]),2)*cos(2*q[3])*sin(q[1])) + 
            pow(cos(q[3]),2)*(-(dq[3]*(dq[2] + dq[0]*cos(q[1]))*sin(q[2])) + dq[0]*dq[2]*sin(q[1])*pow(sin(q[2]),2) + dq[1]*dq[2]*sin(2*q[2])) + pow(dq[0],2)*pow(cos(q[2]),3)*cos(q[3])*pow(sin(q[1]),2)*sin(q[3]) - 2*dq[1]*dq[3]*cos(q[3])*pow(sin(q[2]),2)*sin(q[3]) - 
            cos(q[2])*cos(q[3])*(pow(dq[2],2) + 2*dq[0]*dq[2]*cos(q[1]) + pow(dq[0],2)*pow(cos(q[1]),2) - 2*dq[0]*dq[3]*sin(q[1])*sin(q[2]) - pow(dq[0],2)*pow(sin(q[1]),2)*pow(sin(q[2]),2))*sin(q[3]) + 
            (dq[2] + dq[0]*cos(q[1]))*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*pow(sin(q[3]),2)) + (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*
        (-8*cos(q[3])*(8*dq[4]*dq[5]*cos(q[4])*sin(q[2]) + 8*dq[4]*pow(cos(q[4]),2)*(dq[3]*sin(q[2]) + dq[0]*sin(q[1])*pow(sin(q[2]),2) + dq[1]*sin(2*q[2])) + 9*dq[1]*dq[3]*sin(q[3]) - 8*dq[3]*dq[4]*sin(q[2])*pow(sin(q[4]),2) - 
                8*dq[0]*dq[4]*sin(q[1])*pow(sin(q[2]),2)*pow(sin(q[4]),2) - 8*dq[1]*dq[4]*sin(2*q[2])*pow(sin(q[4]),2) - 8*dq[1]*dq[2]*pow(sin(q[2]),2)*sin(2*q[4]) + 8*dq[0]*dq[2]*sin(q[1])*sin(2*q[2])*sin(2*q[4]) + 2*pow(dq[0],2)*sin(2*q[1])*sin(2*q[2])*sin(2*q[4])) + 
            pow(cos(q[3]),2)*(-4*pow(dq[0],2)*(5 + cos(2*q[2]))*sin(2*q[1])*pow(sin(q[4]),2) - 64*dq[3]*(dq[2] + dq[0]*cos(q[1]))*sin(q[2])*pow(sin(q[4]),2) - 8*dq[1]*(dq[2]*cos(2*q[4])*sin(2*q[2]) + dq[4]*(-3 + cos(2*q[2]))*sin(2*q[4]))) + 
            2*pow(cos(q[2]),2)*(dq[0]*(4*dq[2] - 10*dq[2]*cos(2*q[3]) - 8*(dq[3] - 2*dq[4])*cos(q[3] - 2*q[4]) + 7*dq[2]*cos(2*(q[3] - q[4])) + 28*dq[2]*cos(2*q[4]) + 7*dq[2]*cos(2*(q[3] + q[4])) + 8*dq[3]*cos(q[3] + 2*q[4]) + 16*dq[4]*cos(q[3] + 2*q[4]))*sin(q[1]) + 
                pow(dq[0],2)*(2 - 2*cos(2*q[3]) + cos(2*(q[3] - q[4])) + 6*cos(2*q[4]) + cos(2*(q[3] + q[4])))*sin(2*q[1]) - 8*dq[1]*(8*dq[2]*cos(q[3])*cos(q[4])*sin(q[4]) - 2*dq[3]*sin(2*q[3])*pow(sin(q[4]),2) + dq[4]*(3 + cos(2*q[3]))*sin(2*q[4]))) + 
            8*cos(q[2])*(8*dq[2]*dq[4]*pow(cos(q[4]),2)*sin(q[3]) + 6*pow(dq[0],2)*cos(q[3])*pow(sin(q[1]),2)*sin(q[3]) - 2*pow(dq[2],2)*sin(2*q[3]) + 2*pow(dq[2],2)*pow(cos(q[4]),2)*sin(2*q[3]) + 2*pow(dq[0],2)*cos(2*q[1])*pow(cos(q[4]),2)*sin(2*q[3]) + 
                pow(dq[0],2)*pow(cos(q[1]),2)*(-3 + cos(2*q[4]))*sin(2*q[3]) - pow(dq[0],2)*cos(2*q[4])*pow(sin(q[1]),2)*sin(2*q[3]) - 8*dq[4]*dq[5]*sin(q[4]) - 8*dq[2]*dq[5]*cos(q[3])*sin(q[4]) - 8*dq[2]*dq[4]*sin(q[3])*pow(sin(q[4]),2) - 
                2*pow(dq[2],2)*sin(2*q[3])*pow(sin(q[4]),2) + 8*dq[0]*cos(q[1])*(dq[4]*cos(2*q[4])*sin(q[3]) - cos(q[3])*sin(q[4])*(dq[5] + 2*dq[2]*sin(q[3])*sin(q[4]))) - 8*dq[3]*dq[4]*sin(2*q[4]) - 
                2*sin(q[2])*(2*dq[1]*dq[2] + dq[1]*dq[2]*pow(cos(q[3]),2)*(-2 + cos(2*q[4])) + 4*dq[0]*dq[3]*cos(q[3])*pow(cos(q[4]),2)*sin(q[1])*sin(q[3]) - dq[1]*dq[2]*(-2 + cos(2*q[4]))*pow(sin(q[3]),2) - 3*dq[0]*dq[3]*sin(q[1])*sin(2*q[3]) + 
                2*dq[0]*dq[4]*cos(2*q[3])*sin(q[1])*sin(2*q[4]))) + 4*(-(pow(dq[0],2)*sin(2*q[1])) - 8*dq[2]*dq[3]*sin(q[2]) - 8*dq[0]*dq[3]*cos(q[1])*sin(q[2]) - 16*dq[5]*(dq[2] + dq[0]*cos(q[1]))*cos(q[4])*sin(q[2]) + 3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[3]),2) + 
                8*dq[2]*dq[3]*sin(q[2])*pow(sin(q[3]),2) + 8*dq[0]*dq[3]*cos(q[1])*sin(q[2])*pow(sin(q[3]),2) + 2*dq[1]*dq[2]*cos(2*q[4])*sin(2*q[2])*pow(sin(q[3]),2) + 3*dq[1]*dq[3]*cos(2*q[2])*sin(2*q[3]) + 3*dq[1]*dq[3]*cos(2*q[4])*sin(2*q[3]) - 
                dq[1]*dq[3]*cos(2*q[2])*cos(2*q[4])*sin(2*q[3]) + pow(cos(q[4]),2)*(dq[0]*dq[2]*(-11 + 5*cos(2*q[2]))*sin(q[1]) + pow(dq[0],2)*sin(2*q[1])*(pow(sin(q[2]),2)*(-3 + pow(sin(q[3]),2)) - 3*(1 + pow(sin(q[3]),2))) + 
                4*(dq[3]*(dq[2] + dq[0]*cos(q[1]))*(-3 + cos(2*q[3]))*sin(q[2]) - 3*dq[1]*dq[2]*sin(2*q[2]) + dq[1]*dq[3]*sin(2*q[3]) + dq[1]*dq[3]*pow(sin(q[2]),2)*sin(2*q[3]))) + 16*dq[3]*dq[5]*sin(q[2])*sin(q[3])*sin(q[4]) + 3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[4]),2) + 
                8*dq[2]*dq[3]*sin(q[2])*pow(sin(q[4]),2) + 8*dq[0]*dq[3]*cos(q[1])*sin(q[2])*pow(sin(q[4]),2) + 3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[2]),2)*pow(sin(q[4]),2) + 12*dq[1]*dq[2]*sin(2*q[2])*pow(sin(q[4]),2) + 
                3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 8*dq[2]*dq[3]*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 8*dq[0]*dq[3]*cos(q[1])*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) - 4*dq[1]*dq[4]*sin(2*q[4]) + 
                12*dq[1]*dq[4]*pow(sin(q[2]),2)*sin(2*q[4]) - 8*pow(dq[2],2)*sin(q[2])*sin(q[3])*sin(2*q[4]) + 8*pow(dq[3],2)*sin(q[2])*sin(q[3])*sin(2*q[4]) - 16*dq[0]*dq[2]*cos(q[1])*sin(q[2])*sin(q[3])*sin(2*q[4]) - 
                8*pow(dq[0],2)*pow(cos(q[1]),2)*sin(q[2])*sin(q[3])*sin(2*q[4]) + 8*pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])*sin(q[3])*sin(2*q[4]) + 8*dq[1]*dq[3]*sin(2*q[2])*sin(q[3])*sin(2*q[4]) - 4*dq[1]*dq[4]*pow(sin(q[3]),2)*sin(2*q[4]) - 
                4*dq[1]*dq[4]*pow(sin(q[2]),2)*pow(sin(q[3]),2)*sin(2*q[4]) - 8*dq[2]*dq[4]*sin(q[2])*sin(2*q[3])*sin(2*q[4]) - 8*dq[0]*dq[4]*cos(q[1])*sin(q[2])*sin(2*q[3])*sin(2*q[4]) - 
                2*dq[0]*sin(q[1])*(-2*(4*(dq[5] + dq[3]*cos(q[4]))*sin(q[3])*sin(q[4]) + dq[2]*(1 + pow(sin(q[4]),2)) + dq[2]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2))) + sin(2*q[2])*(dq[3]*cos(2*q[4])*sin(2*q[3]) + 6*dq[4]*sin(2*q[4])) + 
                pow(sin(q[2]),2)*(cos(q[1])*(dq[0] + dq[0]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2))) + 2*(dq[2] - 3*dq[2]*pow(sin(q[4]),2) + dq[2]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2)) - 2*dq[3]*sin(q[3])*sin(2*q[4])))))) + 
        (LL6*m6 + L6*(m6 + m7))*(-8*cos(q[3])*(8*dq[4]*dq[5]*cos(q[4])*sin(q[2]) + 8*dq[4]*pow(cos(q[4]),2)*(dq[3]*sin(q[2]) + dq[0]*sin(q[1])*pow(sin(q[2]),2) + dq[1]*sin(2*q[2])) + 9*dq[1]*dq[3]*sin(q[3]) - 8*dq[3]*dq[4]*sin(q[2])*pow(sin(q[4]),2) - 
                8*dq[0]*dq[4]*sin(q[1])*pow(sin(q[2]),2)*pow(sin(q[4]),2) - 8*dq[1]*dq[4]*sin(2*q[2])*pow(sin(q[4]),2) - 8*dq[1]*dq[2]*pow(sin(q[2]),2)*sin(2*q[4]) + 8*dq[0]*dq[2]*sin(q[1])*sin(2*q[2])*sin(2*q[4]) + 2*pow(dq[0],2)*sin(2*q[1])*sin(2*q[2])*sin(2*q[4])) + 
            pow(cos(q[3]),2)*(-4*pow(dq[0],2)*(5 + cos(2*q[2]))*sin(2*q[1])*pow(sin(q[4]),2) - 64*dq[3]*(dq[2] + dq[0]*cos(q[1]))*sin(q[2])*pow(sin(q[4]),2) - 8*dq[1]*(dq[2]*cos(2*q[4])*sin(2*q[2]) + dq[4]*(-3 + cos(2*q[2]))*sin(2*q[4]))) + 
            2*pow(cos(q[2]),2)*(dq[0]*(4*dq[2] - 10*dq[2]*cos(2*q[3]) - 8*(dq[3] - 2*dq[4])*cos(q[3] - 2*q[4]) + 7*dq[2]*cos(2*(q[3] - q[4])) + 28*dq[2]*cos(2*q[4]) + 7*dq[2]*cos(2*(q[3] + q[4])) + 8*dq[3]*cos(q[3] + 2*q[4]) + 16*dq[4]*cos(q[3] + 2*q[4]))*sin(q[1]) + 
                pow(dq[0],2)*(2 - 2*cos(2*q[3]) + cos(2*(q[3] - q[4])) + 6*cos(2*q[4]) + cos(2*(q[3] + q[4])))*sin(2*q[1]) - 8*dq[1]*(8*dq[2]*cos(q[3])*cos(q[4])*sin(q[4]) - 2*dq[3]*sin(2*q[3])*pow(sin(q[4]),2) + dq[4]*(3 + cos(2*q[3]))*sin(2*q[4]))) + 
            8*cos(q[2])*(8*dq[2]*dq[4]*pow(cos(q[4]),2)*sin(q[3]) + 6*pow(dq[0],2)*cos(q[3])*pow(sin(q[1]),2)*sin(q[3]) - 2*pow(dq[2],2)*sin(2*q[3]) + 2*pow(dq[2],2)*pow(cos(q[4]),2)*sin(2*q[3]) + 2*pow(dq[0],2)*cos(2*q[1])*pow(cos(q[4]),2)*sin(2*q[3]) + 
                pow(dq[0],2)*pow(cos(q[1]),2)*(-3 + cos(2*q[4]))*sin(2*q[3]) - pow(dq[0],2)*cos(2*q[4])*pow(sin(q[1]),2)*sin(2*q[3]) - 8*dq[4]*dq[5]*sin(q[4]) - 8*dq[2]*dq[5]*cos(q[3])*sin(q[4]) - 8*dq[2]*dq[4]*sin(q[3])*pow(sin(q[4]),2) - 
                2*pow(dq[2],2)*sin(2*q[3])*pow(sin(q[4]),2) + 8*dq[0]*cos(q[1])*(dq[4]*cos(2*q[4])*sin(q[3]) - cos(q[3])*sin(q[4])*(dq[5] + 2*dq[2]*sin(q[3])*sin(q[4]))) - 8*dq[3]*dq[4]*sin(2*q[4]) - 
                2*sin(q[2])*(2*dq[1]*dq[2] + dq[1]*dq[2]*pow(cos(q[3]),2)*(-2 + cos(2*q[4])) + 4*dq[0]*dq[3]*cos(q[3])*pow(cos(q[4]),2)*sin(q[1])*sin(q[3]) - dq[1]*dq[2]*(-2 + cos(2*q[4]))*pow(sin(q[3]),2) - 3*dq[0]*dq[3]*sin(q[1])*sin(2*q[3]) + 
                2*dq[0]*dq[4]*cos(2*q[3])*sin(q[1])*sin(2*q[4]))) + 4*(-(pow(dq[0],2)*sin(2*q[1])) - 8*dq[2]*dq[3]*sin(q[2]) - 8*dq[0]*dq[3]*cos(q[1])*sin(q[2]) - 16*dq[5]*(dq[2] + dq[0]*cos(q[1]))*cos(q[4])*sin(q[2]) + 3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[3]),2) + 
                8*dq[2]*dq[3]*sin(q[2])*pow(sin(q[3]),2) + 8*dq[0]*dq[3]*cos(q[1])*sin(q[2])*pow(sin(q[3]),2) + 2*dq[1]*dq[2]*cos(2*q[4])*sin(2*q[2])*pow(sin(q[3]),2) + 3*dq[1]*dq[3]*cos(2*q[2])*sin(2*q[3]) + 3*dq[1]*dq[3]*cos(2*q[4])*sin(2*q[3]) - 
                dq[1]*dq[3]*cos(2*q[2])*cos(2*q[4])*sin(2*q[3]) + pow(cos(q[4]),2)*(dq[0]*dq[2]*(-11 + 5*cos(2*q[2]))*sin(q[1]) + pow(dq[0],2)*sin(2*q[1])*(pow(sin(q[2]),2)*(-3 + pow(sin(q[3]),2)) - 3*(1 + pow(sin(q[3]),2))) + 
                4*(dq[3]*(dq[2] + dq[0]*cos(q[1]))*(-3 + cos(2*q[3]))*sin(q[2]) - 3*dq[1]*dq[2]*sin(2*q[2]) + dq[1]*dq[3]*sin(2*q[3]) + dq[1]*dq[3]*pow(sin(q[2]),2)*sin(2*q[3]))) + 16*dq[3]*dq[5]*sin(q[2])*sin(q[3])*sin(q[4]) + 3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[4]),2) + 
                8*dq[2]*dq[3]*sin(q[2])*pow(sin(q[4]),2) + 8*dq[0]*dq[3]*cos(q[1])*sin(q[2])*pow(sin(q[4]),2) + 3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[2]),2)*pow(sin(q[4]),2) + 12*dq[1]*dq[2]*sin(2*q[2])*pow(sin(q[4]),2) + 
                3*pow(dq[0],2)*sin(2*q[1])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 8*dq[2]*dq[3]*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 8*dq[0]*dq[3]*cos(q[1])*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) - 4*dq[1]*dq[4]*sin(2*q[4]) + 
                12*dq[1]*dq[4]*pow(sin(q[2]),2)*sin(2*q[4]) - 8*pow(dq[2],2)*sin(q[2])*sin(q[3])*sin(2*q[4]) + 8*pow(dq[3],2)*sin(q[2])*sin(q[3])*sin(2*q[4]) - 16*dq[0]*dq[2]*cos(q[1])*sin(q[2])*sin(q[3])*sin(2*q[4]) - 
                8*pow(dq[0],2)*pow(cos(q[1]),2)*sin(q[2])*sin(q[3])*sin(2*q[4]) + 8*pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])*sin(q[3])*sin(2*q[4]) + 8*dq[1]*dq[3]*sin(2*q[2])*sin(q[3])*sin(2*q[4]) - 4*dq[1]*dq[4]*pow(sin(q[3]),2)*sin(2*q[4]) - 
                4*dq[1]*dq[4]*pow(sin(q[2]),2)*pow(sin(q[3]),2)*sin(2*q[4]) - 8*dq[2]*dq[4]*sin(q[2])*sin(2*q[3])*sin(2*q[4]) - 8*dq[0]*dq[4]*cos(q[1])*sin(q[2])*sin(2*q[3])*sin(2*q[4]) - 
                2*dq[0]*sin(q[1])*(-2*(4*(dq[5] + dq[3]*cos(q[4]))*sin(q[3])*sin(q[4]) + dq[2]*(1 + pow(sin(q[4]),2)) + dq[2]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2))) + sin(2*q[2])*(dq[3]*cos(2*q[4])*sin(2*q[3]) + 6*dq[4]*sin(2*q[4])) + 
                pow(sin(q[2]),2)*(cos(q[1])*(dq[0] + dq[0]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2))) + 2*(dq[2] - 3*dq[2]*pow(sin(q[4]),2) + dq[2]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2)) - 2*dq[3]*sin(q[3])*sin(2*q[4])))))))/64.);


    Coriolis_Comp(2,0)=(-(dq[0]*dq[1]*im3*kr3*sin(q[1])) + (ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(-(dq[0]*cos(q[2])*sin(q[1])) + dq[1]*sin(q[2]))*(dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2])) + 
        dq[0]*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*cos(q[1])*(2*dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2]))*sin(q[3]) + 
        (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(dq[0]*dq[1]*pow(cos(q[2]),2)*pow(cos(q[3]),2)*sin(q[1]) - dq[1]*pow(cos(q[3]),2)*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])) + 
        pow(dq[0],2)*cos(q[1])*cos(q[3])*sin(q[1])*sin(q[2])*sin(q[3]) - dq[0]*dq[1]*sin(q[1])*pow(sin(q[3]),2) + dq[1]*dq[3]*sin(q[2])*pow(sin(q[3]),2) + dq[2]*dq[3]*sin(2*q[3]) + dq[0]*dq[3]*cos(q[1])*sin(2*q[3]) + 
        cos(q[2])*(pow(cos(q[3]),2)*(dq[0]*dq[3]*sin(q[1]) - pow(dq[1],2)*sin(q[2]) + pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])) - dq[0]*dq[3]*sin(q[1])*pow(sin(q[3]),2) + dq[0]*dq[1]*cos(q[1])*sin(2*q[3]))) + 
        im6*kr6*(dq[5]*cos(q[4])*(dq[1]*sin(q[2]) + dq[4]*sin(q[3])) + dq[5]*cos(q[3])*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4]) + cos(q[2])*(-(dq[0]*dq[5]*cos(q[4])*sin(q[1])) + dq[1]*dq[5]*cos(q[3])*sin(q[4]))) + 
        ((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*(64*dq[0]*cos(q[1])*sin(q[3])*sin(q[4])*(cos(q[4])*(dq[1]*sin(q[2]) + dq[4]*sin(q[3])) + (dq[3] + dq[1]*cos(q[2]))*cos(q[3])*sin(q[4])) + 
            pow(dq[0],2)*pow(cos(q[1]),2)*((2 - 2*cos(2*q[3]) + cos(2*(q[3] - q[4])) + 6*cos(2*q[4]) + cos(2*(q[3] + q[4])))*sin(2*q[2]) - 4*cos(q[3])*(cos(q[2])*cos(q[3])*sin(q[2])*pow(sin(q[4]),2) - 2*cos(2*q[2])*sin(2*q[4]))) + 
            2*(2*pow(dq[1],2)*sin(2*q[2]) - 3*pow(dq[0],2)*pow(cos(q[4]),2)*sin(2*q[2]) + 6*pow(dq[1],2)*pow(cos(q[4]),2)*sin(2*q[2]) + 16*dq[4]*dq[5]*cos(q[4])*sin(q[3]) + 16*dq[3]*dq[4]*pow(cos(q[4]),2)*sin(q[3]) + 
                16*dq[1]*dq[4]*cos(q[2])*pow(cos(q[4]),2)*sin(q[3]) + 2*pow(dq[1],2)*sin(2*q[2])*pow(sin(q[3]),2) + pow(dq[0],2)*pow(cos(q[4]),2)*sin(2*q[2])*pow(sin(q[3]),2) + 16*dq[3]*dq[5]*cos(q[3])*sin(q[4]) + 16*dq[1]*dq[5]*cos(q[2])*cos(q[3])*sin(q[4]) + 
                3*pow(dq[0],2)*sin(2*q[2])*pow(sin(q[4]),2) - 6*pow(dq[1],2)*sin(2*q[2])*pow(sin(q[4]),2) + 3*pow(dq[0],2)*pow(cos(q[3]),2)*sin(2*q[2])*pow(sin(q[4]),2) - 4*pow(dq[1],2)*pow(cos(q[3]),2)*sin(2*q[2])*pow(sin(q[4]),2) - 
                16*dq[3]*dq[4]*sin(q[3])*pow(sin(q[4]),2) - 16*dq[1]*dq[4]*cos(q[2])*sin(q[3])*pow(sin(q[4]),2) + 32*dq[2]*dq[3]*cos(q[3])*sin(q[3])*pow(sin(q[4]),2) + 2*pow(dq[1],2)*sin(2*q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 8*dq[2]*dq[4]*sin(2*q[4]) + 
                8*pow(dq[3],2)*cos(q[3])*sin(2*q[4]) + 16*dq[1]*dq[3]*cos(q[2])*cos(q[3])*sin(2*q[4]) - 6*pow(dq[0],2)*pow(cos(q[2]),2)*cos(q[3])*sin(2*q[4]) + 8*pow(dq[1],2)*pow(cos(q[2]),2)*cos(q[3])*sin(2*q[4]) + 
                2*pow(dq[0],2)*cos(2*q[1])*pow(cos(q[2]),2)*cos(q[3])*sin(2*q[4]) - 8*dq[2]*dq[4]*pow(cos(q[3]),2)*sin(2*q[4]) + 4*(pow(dq[0],2) - 2*pow(dq[1],2))*cos(q[3])*pow(sin(q[2]),2)*sin(2*q[4]) - 4*pow(dq[0],2)*cos(q[2])*sin(2*q[1])*sin(q[3])*sin(2*q[4]) + 
                8*dq[2]*dq[4]*pow(sin(q[3]),2)*sin(2*q[4]) - (pow(dq[0],2)*pow(sin(q[1]),2)*((5 + cos(2*q[3]))*pow(cos(q[4]),2)*sin(2*q[2]) - 7*sin(2*q[2])*pow(sin(q[4]),2) - cos(2*q[3])*sin(2*q[2])*pow(sin(q[4]),2) + 
                    4*cos(q[2])*sin(q[2])*(1 + pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2))) - 8*cos(q[3])*pow(sin(q[2]),2)*sin(2*q[4])))/2. - 
                2*sin(q[2])*(-4*dq[1]*dq[3] - 8*dq[1]*dq[5]*cos(q[4]) - 4*dq[1]*dq[3]*pow(sin(q[3]),2) - pow(dq[0],2)*sin(2*q[1])*sin(2*q[3]) + pow(cos(q[4]),2)*(-4*dq[1]*dq[3] + 4*dq[1]*dq[3]*pow(sin(q[3]),2) + pow(dq[0],2)*sin(2*q[1])*sin(2*q[3])) + 
                4*dq[1]*dq[3]*pow(sin(q[4]),2) + 8*dq[1]*dq[3]*pow(cos(q[3]),2)*pow(sin(q[4]),2) - 4*dq[1]*dq[3]*pow(sin(q[3]),2)*pow(sin(q[4]),2) - pow(dq[0],2)*sin(2*q[1])*sin(2*q[3])*pow(sin(q[4]),2) + 
                cos(q[2])*(pow(dq[0],2) + pow(sin(q[3]),2)*(2*pow(dq[1],2)*pow(cos(q[4]),2) + pow(dq[0],2)*(1 + pow(sin(q[4]),2)))) + 4*dq[1]*dq[4]*sin(2*q[3])*sin(2*q[4])) + 
                (dq[0]*sin(q[1])*(-(dq[1]*pow(cos(q[2]),2)*(4 - 10*cos(2*q[3]) + 7*cos(2*(q[3] - q[4])) + 28*cos(2*q[4]) + 7*cos(2*(q[3] + q[4])))) + 4*pow(cos(q[4]),2)*(3*dq[1] + 5*dq[1]*pow(sin(q[2]),2) + 8*dq[4]*sin(q[2])*sin(q[3])) + 
                    32*sin(q[2])*sin(q[4])*(cos(q[3])*(dq[5] + 2*dq[3]*cos(q[4])) - dq[4]*sin(q[3])*sin(q[4])) + 8*dq[1]*pow(sin(q[2]),2)*(1 - 3*pow(sin(q[4]),2) + pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2))) - 
                    8*dq[1]*(1 + pow(sin(q[4]),2) + pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2)) - 4*cos(q[3])*sin(2*q[2])*sin(2*q[4])) - 
                    16*cos(q[2])*(dq[3] + 2*dq[5]*cos(q[4]) + dq[3]*pow(cos(q[3]),2)*pow(cos(q[4]),2) - 2*dq[3]*pow(sin(q[4]),2) - dq[3]*cos(2*q[3])*pow(sin(q[4]),2) + dq[3]*pow(sin(q[3]),2)*(1 + pow(sin(q[4]),2)) - dq[4]*sin(2*q[3])*sin(2*q[4]))))/2.)))/32. + 
        ((LL6*m6 + L6*(m6 + m7))*(cos(q[5])*(pow(dq[0],2)*L4*cos(2*q[3])*sin(2*q[2]) - 32*dq[0]*dq[1]*L4*pow(cos(q[2]),2)*sin(q[1])*pow(sin(q[3]),2) + 
                2*cos(q[2])*(L4*(-2*pow(dq[0],2) + 4*pow(dq[1],2) + 2*pow(dq[0],2)*cos(2*q[1]) - pow(dq[0],2)*cos(2*(q[1] - q[3])) + pow(dq[0],2)*cos(2*q[3]) - 4*pow(dq[1],2)*cos(2*q[3]) - pow(dq[0],2)*cos(2*(q[1] + q[3])))*sin(q[2]) + 
                8*dq[0]*sin(q[3])*(dq[1]*cos(q[1])*(L2 + 2*L4*cos(q[3])) - L4*(2*dq[3] + dq[5]*cos(q[4]))*sin(q[1])*sin(q[3]))) + 
                8*(dq[0]*cos(q[1])*(dq[0]*(L2 + 2*L4*cos(q[3]))*sin(q[1])*sin(q[2])*sin(q[3]) + L4*(2*dq[3] + dq[5]*cos(q[4]))*sin(2*q[3])) + 
                2*L4*(dq[5]*cos(q[4])*sin(q[3])*(dq[4] + dq[2]*cos(q[3]) + dq[1]*sin(q[2])*sin(q[3])) + dq[3]*(2*dq[1]*sin(q[2])*pow(sin(q[3]),2) + dq[2]*sin(2*q[3]))))) + 
            4*(-2*dq[2]*dq[5]*L4 - 2*dq[2]*dq[5]*L4*pow(sin(q[3]),2) - 4*dq[2]*dq[3]*L4*cos(q[4])*pow(sin(q[3]),2) - pow(dq[0],2)*L4*cos(q[4])*sin(2*q[1])*sin(q[2])*pow(sin(q[3]),2) - 2*dq[0]*dq[5]*L4*cos(q[2])*sin(q[1])*sin(2*q[3]) - 
                2*dq[0]*dq[1]*L4*cos(q[4])*sin(q[1])*sin(2*q[3]) - 2*dq[0]*dq[1]*L4*pow(cos(q[2]),2)*cos(q[4])*sin(q[1])*sin(2*q[3]) + 2*dq[1]*dq[5]*L4*sin(q[2])*sin(2*q[3]) + 2*dq[0]*dq[1]*L4*cos(q[4])*sin(q[1])*pow(sin(q[2]),2)*sin(2*q[3]) + 
                pow(dq[1],2)*L4*cos(q[4])*sin(2*q[2])*sin(2*q[3]) + 2*dq[0]*dq[4]*L4*cos(q[2])*sin(q[1])*sin(q[4]) + pow(dq[0],2)*L2*cos(q[2])*sin(2*q[1])*sin(q[4]) - 2*dq[1]*dq[4]*L4*sin(q[2])*sin(q[4]) + 2*pow(dq[3],2)*L4*sin(q[3])*sin(q[4]) - 
                2*pow(dq[4],2)*L4*sin(q[3])*sin(q[4]) - 2*pow(dq[5],2)*L4*sin(q[3])*sin(q[4]) + 4*dq[1]*dq[3]*L4*cos(q[2])*sin(q[3])*sin(q[4]) - pow(dq[0],2)*L4*pow(cos(q[2]),2)*sin(q[3])*sin(q[4]) + 2*pow(dq[1],2)*L4*pow(cos(q[2]),2)*sin(q[3])*sin(q[4]) - 
                pow(dq[0],2)*L4*pow(cos(q[2]),2)*pow(sin(q[1]),2)*sin(q[3])*sin(q[4]) + 4*dq[0]*dq[3]*L4*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + 8*dq[0]*dq[1]*L4*cos(q[2])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + pow(dq[0],2)*L4*pow(sin(q[2]),2)*sin(q[3])*sin(q[4]) - 
                2*pow(dq[1],2)*L4*pow(sin(q[2]),2)*sin(q[3])*sin(q[4]) + pow(dq[0],2)*L4*pow(sin(q[1]),2)*pow(sin(q[2]),2)*sin(q[3])*sin(q[4]) + 2*dq[0]*dq[4]*L4*cos(q[2])*sin(q[1])*pow(sin(q[3]),2)*sin(q[4]) - 2*dq[1]*dq[4]*L4*sin(q[2])*pow(sin(q[3]),2)*sin(q[4]) - 
                2*dq[2]*dq[4]*L4*sin(2*q[3])*sin(q[4]) + pow(dq[0],2)*L4*pow(cos(q[1]),2)*sin(q[3])*(cos(q[3])*cos(q[4])*sin(2*q[2]) + cos(2*q[2])*sin(q[4])) + 
                cos(q[3])*(cos(q[4])*(pow(dq[0],2)*L2*sin(2*q[1])*sin(q[2]) - 2*L4*(-4*dq[1]*dq[3]*sin(q[2]) + dq[0]*cos(q[2])*(4*dq[3]*sin(q[1]) + dq[0]*sin(q[2]) + dq[0]*pow(sin(q[1]),2)*sin(q[2])))*sin(q[3])) + pow(dq[0],2)*L4*cos(q[2])*sin(2*q[1])*sin(q[4])) + 
                2*L4*pow(cos(q[3]),2)*(dq[2]*dq[5] + cos(q[4])*(2*dq[2]*dq[3] + pow(dq[0],2)*cos(q[1])*sin(q[1])*sin(q[2])) - dq[0]*dq[4]*cos(q[2])*sin(q[1])*sin(q[4]) + dq[1]*dq[4]*sin(q[2])*sin(q[4])) + 
                2*dq[0]*cos(q[1])*(-(dq[5]*L4) + L4*pow(cos(q[3]),2)*(dq[5] + 2*(dq[3] + dq[1]*cos(q[2]))*cos(q[4])) - L4*(dq[5] + 2*(dq[3] + dq[1]*cos(q[2]))*cos(q[4]))*pow(sin(q[3]),2) - 2*dq[1]*L2*sin(q[2])*sin(q[4]) - dq[4]*L4*sin(2*q[3])*sin(q[4]) + 
                2*dq[1]*cos(q[3])*(L2*cos(q[2])*cos(q[4]) - L4*sin(q[2])*sin(q[4]))))*sin(q[5])))/8. + ((ia6xx - ia6zz + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*
        (4*pow(cos(q[2]),2)*(pow(dq[0],2)*pow(cos(q[3]),3)*cos(q[4])*pow(cos(q[5]),2)*pow(sin(q[1]),2)*sin(q[4]) + 
                cos(q[3])*cos(q[4])*cos(q[5])*(cos(q[5])*(-pow(dq[1],2) + pow(dq[0],2)*pow(sin(q[1]),2)*pow(sin(q[3]),2))*sin(q[4]) - 2*dq[0]*dq[1]*sin(q[1])*sin(q[3])*sin(q[5])) + 
                sin(q[3])*(-(dq[0]*dq[1]*pow(cos(q[5]),2)*sin(q[1])*sin(q[3])*pow(sin(q[4]),2)) + cos(q[5])*(pow(dq[1],2) - pow(dq[0],2)*pow(sin(q[1]),2)*pow(sin(q[3]),2))*sin(q[4])*sin(q[5]) + dq[0]*dq[1]*sin(q[1])*sin(q[3])*pow(sin(q[5]),2)) - 
                dq[0]*pow(cos(q[3]),2)*cos(q[5])*sin(q[1])*(-(dq[1]*pow(cos(q[4]),2)*cos(q[5])) + sin(q[4])*(dq[1]*cos(q[5])*sin(q[4]) + dq[0]*sin(q[1])*sin(q[3])*sin(q[5])))) + 
            2*(dq[1]*pow(cos(q[3]),3)*cos(q[5])*sin(q[2])*(dq[1]*cos(q[5])*sin(q[2])*sin(2*q[4]) - dq[0]*cos(q[1])*sin(q[4])*sin(q[5])) - 
                2*pow(cos(q[3]),2)*(dq[1]*pow(cos(q[4]),2)*pow(cos(q[5]),2)*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])) - dq[1]*pow(cos(q[5]),2)*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*pow(sin(q[4]),2) + 
                cos(q[5])*(-2*dq[2]*dq[5] - 2*dq[0]*dq[5]*cos(q[1]) + pow(dq[1],2)*pow(sin(q[2]),2)*sin(q[3])*sin(q[4]))*sin(q[5]) + dq[1]*(dq[0]*sin(q[1]) - dq[3]*sin(q[2]))*pow(sin(q[5]),2) + 
                cos(q[4])*(dq[1]*pow(cos(q[5]),2)*sin(q[2])*(dq[5] + (dq[2] + dq[0]*cos(q[1]))*sin(q[3])*sin(q[4])) - cos(q[5])*(2*dq[2]*dq[3] + dq[0]*cos(q[1])*(2*dq[3] + dq[0]*sin(q[1])*sin(q[2])))*sin(q[5]) - dq[1]*dq[5]*sin(q[2])*pow(sin(q[5]),2))) + 
                cos(q[3])*(pow(cos(q[5]),2)*(-2*dq[5]*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4]) + pow(dq[1],2)*pow(sin(q[2]),2)*pow(sin(q[3]),2)*sin(2*q[4])) + dq[1]*pow(cos(q[4]),3)*cos(q[5])*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[3])*sin(q[5]) + 
                cos(q[5])*(-(dq[0]*cos(q[1])*(4*dq[4]*sin(q[3]) + dq[1]*sin(q[2])*(3 + pow(sin(q[3]),2)))*sin(q[4])) + 4*dq[4]*(dq[5] - dq[2]*sin(q[3])*sin(q[4])))*sin(q[5]) - 
                2*(-(dq[5]*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4])) + sin(q[3])*(dq[2]*dq[3]*(1 + pow(sin(q[4]),2)) + dq[0]*cos(q[1])*(dq[3] + (dq[3] + dq[0]*sin(q[1])*sin(q[2]))*pow(sin(q[4]),2))))*pow(sin(q[5]),2) + 
                pow(cos(q[4]),2)*sin(q[3])*(pow(cos(q[5]),2)*(4*dq[2]*dq[3] + 4*dq[0]*dq[3]*cos(q[1]) + pow(dq[0],2)*sin(2*q[1])*sin(q[2])) - 2*(dq[2]*dq[3] + dq[0]*cos(q[1])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])))*pow(sin(q[5]),2)) + 
                cos(q[4])*(pow(cos(q[5]),2)*(4*dq[5]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3]) - 2*pow(dq[3] + dq[0]*sin(q[1])*sin(q[2]),2)*sin(q[4])) + 
                    dq[1]*cos(q[5])*sin(q[3])*(dq[3]*sin(q[2])*(7 + pow(sin(q[4]),2)) + dq[0]*sin(q[1])*(-4 + pow(sin(q[2]),2)*(3 + pow(sin(q[4]),2))))*sin(q[5]) - 4*dq[5]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])*pow(sin(q[5]),2))) - 
                2*sin(q[3])*(-(pow(cos(q[5]),2)*((dq[3] + dq[0]*sin(q[1])*sin(q[2]))*(dq[4] + dq[1]*sin(q[2])*sin(q[3]))*pow(sin(q[4]),2) - dq[4]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])*sin(2*q[4]))) + 
                pow(cos(q[4]),3)*cos(q[5])*(dq[2]*dq[3] + dq[0]*cos(q[1])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])))*sin(q[3])*sin(q[5]) + 
                cos(q[5])*(-2*dq[1]*dq[5]*cos(q[3])*sin(q[2]) + sin(q[4])*(pow(dq[4],2) - pow(dq[3],2)*pow(sin(q[4]),2) - 2*dq[0]*dq[3]*sin(q[1])*sin(q[2])*pow(sin(q[4]),2) + 
                        pow(sin(q[2]),2)*(pow(dq[1],2)*pow(sin(q[3]),2) - pow(dq[0],2)*pow(sin(q[1]),2)*pow(sin(q[4]),2))))*sin(q[5]) + 
                cos(q[4])*(pow(cos(q[5]),2)*(-(dq[4]*dq[5]) + dq[1]*sin(q[2])*(-(dq[5]*sin(q[3])) - (dq[2] - dq[0]*cos(q[1]) + 2*dq[4]*cos(q[3]))*sin(q[4]) + (dq[2] + dq[0]*cos(q[1]))*pow(sin(q[3]),2)*sin(q[4]))) + 
                    cos(q[5])*(-2*dq[5]*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4]) + sin(q[3])*(dq[2]*dq[3]*(1 + pow(sin(q[4]),2)) + dq[0]*cos(q[1])*(dq[3] + (dq[3] + dq[0]*sin(q[1])*sin(q[2]))*pow(sin(q[4]),2))))*sin(q[5]) + 
                    dq[5]*(dq[4] + dq[1]*sin(q[2])*sin(q[3]))*pow(sin(q[5]),2)) + pow(cos(q[4]),2)*(pow(cos(q[5]),2)*(dq[0]*sin(q[1])*(dq[4]*sin(q[2]) + dq[1]*sin(q[3])) + dq[3]*(dq[4] - dq[1]*sin(q[2])*sin(q[3]))) - 
                    cos(q[5])*(2*dq[1]*dq[5]*cos(q[3])*sin(q[2]) + pow(dq[3] + dq[0]*sin(q[1])*sin(q[2]),2)*sin(q[4]))*sin(q[5]) + (dq[3] + dq[0]*sin(q[1])*sin(q[2]))*(dq[4] + dq[1]*sin(q[2])*sin(q[3]))*pow(sin(q[5]),2) + dq[5]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])*sin(2*q[5])) + 
                sin(q[4])*((dq[3] + dq[0]*sin(q[1])*sin(q[2]))*(dq[4] + dq[1]*sin(q[2])*sin(q[3]))*sin(q[4])*pow(sin(q[5]),2) + dq[1]*dq[4]*sin(q[2])*sin(q[3])*sin(2*q[5])))) + 
            cos(q[2])*(dq[0]*dq[1]*cos(q[1])*cos(2*q[4])*sin(2*q[3]) + 3*dq[0]*dq[1]*cos(q[1])*cos(2*q[5])*sin(2*q[3]) + 4*dq[1]*dq[4]*pow(cos(q[5]),2)*sin(q[3])*pow(sin(q[4]),2) - 4*dq[0]*dq[3]*pow(cos(q[5]),2)*sin(q[1])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 
                4*pow(dq[1],2)*pow(cos(q[5]),2)*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) - 4*pow(dq[0],2)*pow(cos(q[5]),2)*pow(sin(q[1]),2)*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2) + 
                pow(dq[0],2)*pow(cos(q[5]),2)*sin(2*q[1])*pow(sin(q[3]),3)*sin(2*q[4]) + 8*dq[0]*dq[1]*cos(q[5])*sin(q[1])*sin(q[2])*pow(sin(q[3]),3)*sin(q[4])*sin(q[5]) + 8*dq[0]*dq[1]*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])*pow(sin(q[4]),3)*sin(q[5]) - 
                4*dq[1]*dq[4]*sin(q[3])*pow(sin(q[4]),2)*pow(sin(q[5]),2) + 4*dq[0]*dq[3]*sin(q[1])*pow(sin(q[3]),2)*pow(sin(q[4]),2)*pow(sin(q[5]),2) - 4*pow(dq[1],2)*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2)*pow(sin(q[5]),2) + 
                4*pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[4]),2)*pow(sin(q[5]),2) - 
                4*pow(cos(q[4]),2)*sin(q[3])*(pow(cos(q[5]),2)*(dq[1]*dq[4] + dq[0]*dq[3]*sin(q[1])*sin(q[3])) - 2*dq[1]*cos(q[5])*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4])*sin(q[5]) + 
                (-(dq[0]*dq[3]*sin(q[1])*sin(q[3])) - pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])*sin(q[3]) + dq[1]*(dq[4] + dq[1]*sin(q[2])*sin(q[3])))*pow(sin(q[5]),2)) + 
                dq[1]*pow(cos(q[4]),3)*(-((dq[2] + dq[0]*cos(q[1]))*pow(sin(q[3]),2)) + dq[1]*sin(q[2])*sin(2*q[3]))*sin(2*q[5]) + 4*dq[0]*dq[4]*sin(q[1])*pow(sin(q[3]),2)*sin(q[4])*sin(2*q[5]) + dq[0]*dq[2]*sin(q[1])*sin(q[3])*sin(2*q[3])*sin(q[4])*sin(2*q[5]) + 
                4*dq[1]*dq[3]*sin(q[3])*pow(sin(q[4]),3)*sin(2*q[5]) + 2*dq[0]*pow(cos(q[3]),3)*sin(q[1])*sin(q[4])*(-4*dq[1]*cos(q[4])*pow(cos(q[5]),2)*sin(q[2]) + (dq[2] + dq[0]*cos(q[1]))*sin(2*q[5])) + 
                cos(q[4])*(-4*dq[0]*pow(cos(q[5]),2)*sin(q[1])*sin(q[3])*(dq[5]*sin(q[3]) + dq[2]*sin(q[4]) - dq[2]*pow(sin(q[3]),2)*sin(q[4])) + 
                dq[1]*cos(q[5])*sin(q[3])*(dq[0]*cos(q[1])*(-7 + cos(2*q[4]))*sin(q[3]) - 2*sin(q[4])*(-4*dq[5] + dq[2]*sin(q[3])*sin(q[4])))*sin(q[5]) + dq[1]*(dq[2]*pow(sin(q[3]),2) + dq[1]*sin(q[2])*sin(2*q[3])*(1 + pow(sin(q[4]),2)))*sin(2*q[5]) + 
                4*dq[0]*sin(q[1])*(dq[5]*pow(sin(q[3]),2)*pow(sin(q[5]),2) - dq[3]*sin(2*q[3])*sin(2*q[5]))) + 
                4*pow(cos(q[3]),2)*(pow(cos(q[4]),2)*pow(cos(q[5]),2)*(dq[0]*dq[3]*sin(q[1]) - pow(dq[1],2)*sin(q[2]) + pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])) - 
                pow(cos(q[5]),2)*(dq[0]*dq[3]*sin(q[1]) - pow(dq[1],2)*sin(q[2]) + pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2]))*pow(sin(q[4]),2) + 
                dq[0]*cos(q[4])*(pow(cos(q[5]),2)*sin(q[1])*(dq[5] + (dq[2] + dq[0]*cos(q[1]))*sin(q[3])*sin(q[4])) - dq[5]*sin(q[1])*pow(sin(q[5]),2) + dq[1]*cos(q[1])*sin(2*q[5])) + dq[0]*sin(q[1])*(-(dq[3]*pow(sin(q[5]),2)) + dq[1]*sin(q[2])*sin(q[3])*sin(q[4])*sin(2*q[5])))
                + cos(q[3])*(dq[0]*dq[1]*cos(q[1])*(-2 + cos(2*(q[4] - q[5])) + cos(2*(q[4] + q[5])))*sin(q[3]) - 2*
                    (2*pow(cos(q[5]),2)*(dq[1]*dq[5] + 2*cos(q[4])*(dq[1]*dq[3] + dq[0]*sin(q[1])*(dq[4]*sin(q[3]) + dq[1]*sin(q[2])*(1 + pow(sin(q[3]),2)))))*sin(q[4]) + 
                    dq[0]*cos(q[5])*sin(q[1])*sin(q[3])*(5*dq[5] + 2*dq[5]*pow(cos(q[4]),2) + dq[5]*cos(2*q[4]) + 4*dq[0]*cos(q[4])*sin(q[1])*sin(q[2]) - 2*dq[0]*cos(q[1])*sin(q[3])*sin(q[4]))*sin(q[5]) + 
                    sin(q[4])*(-2*dq[1]*dq[5]*pow(sin(q[5]),2) + dq[0]*dq[2]*sin(q[1])*sin(2*q[5])))))))/4.);


    Coriolis_Comp(3,0)=(im4*kr4*(dq[0]*dq[2]*cos(q[2])*sin(q[1]) + dq[1]*(-dq[2] + dq[0]*cos(q[1]))*sin(q[2])) + (ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(dq[0]*dq[2]*cos(q[2])*sin(q[1]) + dq[1]*(-dq[2] + dq[0]*cos(q[1]))*sin(q[2])) - 
        (L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*(pow(dq[0],2)*cos(q[2])*cos(q[3])*sin(2*q[1]) - 4*dq[0]*dq[1]*cos(q[1])*cos(q[3])*sin(q[2]) + pow(dq[0],2)*pow(cos(q[1]),2)*sin(q[3]) - (pow(dq[0],2) + 2*pow(dq[1],2) + pow(dq[0],2)*pow(sin(q[1]),2))*sin(q[3])))/2. + 
        (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(dq[1]*(dq[2] + dq[0]*cos(q[1]))*pow(cos(q[3]),2)*sin(q[2]) + pow(dq[0],2)*pow(cos(q[2]),2)*cos(q[3])*pow(sin(q[1]),2)*sin(q[3]) - 
        cos(q[3])*(pow(dq[2],2) + 2*dq[0]*dq[2]*cos(q[1]) + pow(dq[0],2)*pow(cos(q[1]),2) - pow(dq[1],2)*pow(sin(q[2]),2))*sin(q[3]) - dq[1]*(dq[2] + dq[0]*cos(q[1]))*sin(q[2])*pow(sin(q[3]),2) - 
        dq[0]*cos(q[2])*sin(q[1])*((dq[2] + dq[0]*cos(q[1]))*pow(cos(q[3]),2) - (dq[2] + dq[0]*cos(q[1]))*pow(sin(q[3]),2) + dq[1]*sin(q[2])*sin(2*q[3]))) + 
        (ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*(pow(cos(q[4]),2)*(dq[0]*cos(q[2])*(dq[2] + dq[4]*cos(q[3]))*sin(q[1]) - dq[1]*(dq[2] - dq[0]*cos(q[1]) + dq[4]*cos(q[3]))*sin(q[2]) + dq[4]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])) - 
        (cos(q[4])*(4*dq[3]*dq[4] - 2*dq[0]*dq[1]*pow(cos(q[2]),2)*sin(q[1])*sin(q[3]) + pow(dq[1],2)*sin(2*q[2])*sin(q[3]) + cos(q[2])*(4*dq[1]*dq[4] + 4*dq[1]*dq[2]*cos(q[3]) - 2*pow(dq[0],2)*pow(sin(q[1]),2)*sin(q[2])*sin(q[3])) + 
                2*dq[0]*sin(q[1])*((2*dq[4] + (2*dq[2] + dq[0]*cos(q[1]))*cos(q[3]))*sin(q[2]) + dq[1]*sin(q[3]) + dq[1]*pow(sin(q[2]),2)*sin(q[3])))*sin(q[4]))/2. - 
        (dq[4] + (dq[2] + dq[0]*cos(q[1]))*cos(q[3]) - dq[0]*cos(q[2])*sin(q[1])*sin(q[3]) + dq[1]*sin(q[2])*sin(q[3]))*sin(q[4])*(dq[5] + dq[0]*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - dq[1]*cos(q[3])*sin(q[2])*sin(q[4]) + dq[2]*sin(q[3])*sin(q[4]) + dq[0]*cos(q[1])*sin(q[3])*sin(q[4]))
        ) - ((LL6*m6 + L6*(m6 + m7))*(-4*cos(q[5])*(L4*(-pow(dq[0],2) + 2*pow(dq[1],2) + pow(dq[0],2)*cos(2*q[1]))*pow(sin(q[2]),2)*sin(2*q[3]) - 
                2*((-((pow(dq[0],2) + 2*pow(dq[1],2))*L2) + (pow(dq[0],2) + 4*pow(dq[2],2))*L4*cos(q[3]) + pow(dq[0],2)*cos(2*q[1])*(L2 + 3*L4*cos(q[3])))*sin(q[3]) + L4*(4*dq[0]*dq[2]*cos(q[1]) - dq[1]*(dq[1] - 2*dq[0]*sin(q[1])*sin(2*q[2])))*sin(2*q[3]) + 
                4*dq[5]*L4*(dq[4] + (dq[2] + dq[0]*cos(q[1]))*cos(q[3]))*sin(q[4])) + 8*dq[1]*sin(q[2])*(dq[0]*cos(q[1])*cos(q[3])*(L2 + 2*L4*cos(q[3])) - L4*sin(q[3])*(2*dq[2]*sin(q[3]) + dq[5]*sin(q[4])))) + 
            (cos(q[4])*(L4*(6*pow(dq[0],2) - 12*pow(dq[1],2) + 16*pow(dq[2],2) + 32*dq[0]*dq[2]*cos(q[1]) + 10*pow(dq[0],2)*cos(2*q[1]) + pow(dq[0],2)*cos(2*(q[1] - q[2])) - 2*pow(dq[0],2)*cos(2*q[2]) + 4*pow(dq[1],2)*cos(2*q[2]) + 
                    pow(dq[0],2)*cos(2*(q[1] + q[2])))*pow(cos(q[3]),2) + 4*(4*(pow(dq[4],2) + pow(dq[5],2))*L4 + 8*dq[1]*(dq[4]*L4 + dq[0]*L2*cos(q[1]))*sin(q[2])*sin(q[3]) - 
                    L4*(pow(dq[0],2) - 2*pow(dq[1],2) + 4*pow(dq[2],2) + 8*dq[0]*dq[2]*cos(q[1]) + 3*pow(dq[0],2)*cos(2*q[1]))*pow(sin(q[3]),2) + L4*(-pow(dq[0],2) + 2*pow(dq[1],2) + pow(dq[0],2)*cos(2*q[1]))*pow(sin(q[2]),2)*pow(sin(q[3]),2)) + 
                8*cos(q[3])*(-(pow(dq[0],2)*L2) - 2*pow(dq[1],2)*L2 + 4*dq[2]*dq[4]*L4 + pow(dq[0],2)*L2*cos(2*q[1]) + 8*dq[1]*dq[2]*L4*sin(q[2])*sin(q[3]) + 4*dq[0]*L4*cos(q[1])*(dq[4] + 2*dq[1]*sin(q[2])*sin(q[3])))) + 
                4*L4*(8*dq[3]*dq[5] - ((-pow(dq[0],2) + 4*pow(dq[1],2) + pow(dq[0],2)*cos(2*q[1]))*cos(q[3])*sin(2*q[2])*sin(q[4]))/2. + pow(dq[0],2)*cos(q[3])*pow(sin(q[1]),2)*sin(2*q[2])*sin(q[4]) + 2*pow(dq[0],2)*sin(2*q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + 
                4*dq[0]*sin(q[1])*(-(dq[1]*cos(q[3])*sin(q[4])) - dq[1]*cos(q[3])*pow(sin(q[2]),2)*sin(q[4]) + 2*sin(q[2])*(dq[5] + dq[2]*sin(q[3])*sin(q[4])))))*sin(q[5]) + 
            4*L4*pow(cos(q[2]),2)*((-pow(dq[0],2) + 2*pow(dq[1],2) + pow(dq[0],2)*cos(2*q[1]))*cos(q[5])*sin(2*q[3]) + 
                ((-pow(dq[0],2) + 2*pow(dq[1],2) + pow(dq[0],2)*cos(2*q[1]))*pow(cos(q[3]),2)*cos(q[4]) + (pow(dq[0],2) - 2*pow(dq[1],2) - pow(dq[0],2)*cos(2*q[1]))*cos(q[4])*pow(sin(q[3]),2) + 4*dq[0]*dq[1]*cos(q[3])*sin(q[1])*sin(q[4]))*sin(q[5])) + 
            8*cos(q[2])*(2*dq[0]*cos(q[5])*sin(q[1])*(dq[0]*cos(q[1])*(L2*cos(q[3]) + 2*L4*cos(2*q[3])) - 2*L4*sin(q[3])*(2*dq[2]*sin(q[3]) + dq[5]*sin(q[4]))) + 
                (4*dq[0]*dq[1]*L4*pow(cos(q[3]),2)*cos(q[4])*sin(q[1])*sin(q[2]) - 2*dq[0]*cos(q[4])*sin(q[1])*sin(q[3])*(dq[0]*cos(q[1])*(L2 + 4*L4*cos(q[3])) + 2*L4*(dq[4] + 2*dq[2]*cos(q[3]) + dq[1]*sin(q[2])*sin(q[3]))) + 4*dq[1]*L4*(dq[5] + dq[2]*sin(q[3])*sin(q[4])))*sin(q[5])))
        )/16.);

    Coriolis_Comp(4,0)=((ia5yy + ia6zz)*(cos(q[3])*(-(dq[0]*(dq[1] + dq[3]*cos(q[2]))*sin(q[1])) + dq[1]*dq[3]*sin(q[2])) + (-(dq[0]*cos(q[1])*(dq[3] + dq[1]*cos(q[2]))) + dq[2]*(-dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2])))*sin(q[3])) + 
        im5*kr5*(cos(q[3])*(-(dq[0]*(dq[1] + dq[3]*cos(q[2]))*sin(q[1])) + dq[1]*dq[3]*sin(q[2])) + (-(dq[0]*cos(q[1])*(dq[3] + dq[1]*cos(q[2]))) + dq[2]*(-dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2])))*sin(q[3])) + 
        ((LL6*m6 + L6*(m6 + m7))*(4*dq[2]*L4*pow(cos(q[3]),2)*(dq[0]*cos(q[2])*sin(q[1]) - dq[1]*sin(q[2]))*sin(q[4]) + pow(dq[0],2)*pow(cos(q[1]),2)*sin(q[3])*(L4*cos(q[4])*sin(2*q[2]) + (2*L2 + L4*(3 + cos(2*q[2]))*cos(q[3]))*sin(q[4])) + 
            (L4*cos(q[3])*(16*dq[2]*dq[3]*cos(q[4]) + (pow(dq[0],2)*(-3 + cos(2*q[1]))*pow(cos(q[2]),2) - pow(dq[0],2)*(5 + cos(2*q[2]))*pow(sin(q[1]),2) + 
                    2*(pow(dq[0],2) - 2*pow(dq[1],2) + 4*pow(dq[2],2) + (pow(dq[0],2) - 2*pow(dq[1],2))*pow(sin(q[2]),2)) + 8*dq[0]*dq[1]*sin(q[1])*sin(2*q[2]))*sin(q[3])*sin(q[4])))/2. - 
            2*(L4*cos(q[4])*(4*dq[0]*cos(q[2])*(dq[3] + dq[1]*cos(q[2]))*sin(q[1]) + (-4*dq[1]*dq[3] + (pow(dq[0],2) - 2*pow(dq[1],2))*cos(q[2]))*sin(q[2]) + pow(dq[0],2)*cos(q[2])*pow(sin(q[1]),2)*sin(q[2]))*sin(q[3]) + 
                (-(dq[0]*dq[2]*L4*cos(q[2])*(-3 + cos(2*q[3]))*sin(q[1])) + dq[1]*dq[2]*L4*(-3 + cos(2*q[3]))*sin(q[2]) - pow(dq[1],2)*L4*pow(cos(q[2]),2)*cos(q[3])*sin(q[3]) + L2*(pow(dq[0],2) + 2*pow(dq[1],2) + pow(dq[0],2)*pow(sin(q[1]),2))*sin(q[3]))*sin(q[4])) + 
            4*dq[0]*cos(q[1])*(dq[0]*L2*cos(q[4])*sin(q[1])*sin(q[2]) - dq[1]*L4*sin(q[2])*sin(q[4]) - dq[1]*L4*pow(cos(q[3]),2)*sin(q[2])*sin(q[4]) + dq[1]*L4*sin(q[2])*pow(sin(q[3]),2)*sin(q[4]) + dq[2]*L4*sin(2*q[3])*sin(q[4]) + 
                cos(q[2])*(2*dq[1]*(L2 + L4*cos(q[3]))*cos(q[4]) + dq[0]*(L2*cos(q[3]) + L4*cos(2*q[3]))*sin(q[1])*sin(q[4])) + cos(q[3])*(L4*cos(q[4])*(2*dq[3] + dq[0]*sin(q[1])*sin(q[2])) - 2*dq[1]*L2*sin(q[2])*sin(q[4]))))*sin(q[5]))/4.);

    Coriolis_Comp(5,0)=(im6*kr6*(cos(q[4])*(dq[0]*cos(q[2])*(dq[2] + dq[4]*cos(q[3]))*sin(q[1]) - dq[1]*(dq[2] - dq[0]*cos(q[1]) + dq[4]*cos(q[3]))*sin(q[2]) + dq[4]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])) - 
        (dq[4]*(dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2])) + cos(q[3])*(-(dq[0]*cos(q[1])*(dq[3] + dq[1]*cos(q[2]))) + dq[2]*(-dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2]))) + (dq[0]*(dq[1] + dq[3]*cos(q[2]))*sin(q[1]) - dq[1]*dq[3]*sin(q[2]))*sin(q[3]))*sin(q[4])) + 
        (ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*(cos(q[4])*(dq[0]*cos(q[2])*(dq[2] + dq[4]*cos(q[3]))*sin(q[1]) - dq[1]*(dq[2] - dq[0]*cos(q[1]) + dq[4]*cos(q[3]))*sin(q[2]) + dq[4]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])) - 
        (dq[4]*(dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2])) + cos(q[3])*(-(dq[0]*cos(q[1])*(dq[3] + dq[1]*cos(q[2]))) + dq[2]*(-dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2]))) + (dq[0]*(dq[1] + dq[3]*cos(q[2]))*sin(q[1]) - dq[1]*dq[3]*sin(q[2]))*sin(q[3]))*sin(q[4])) + 
        ((LL6*m6 + L6*(m6 + m7))*(cos(q[4])*cos(q[5])*(4*L2*(pow(dq[0],2) + 2*pow(dq[1],2) - pow(dq[0],2)*cos(2*q[1]))*sin(q[3]) + 8*sin(q[2])*(2*dq[0]*dq[1]*cos(q[1])*cos(q[3])*(L2 + L4*cos(q[3])) - 2*dq[1]*dq[2]*L4*pow(sin(q[3]),2)) + 
                4*dq[0]*cos(q[2])*(-(dq[0]*(L2*cos(q[3]) + L4*cos(2*q[3]))*sin(2*q[1])) + 4*dq[2]*L4*sin(q[1])*pow(sin(q[3]),2)) - 
                L4*(pow(dq[0],2) - 2*pow(dq[1],2) + 4*pow(dq[2],2) + 8*dq[0]*dq[2]*cos(q[1]) - (pow(dq[0],2) - 2*pow(dq[1],2))*cos(2*q[2]) + pow(dq[0],2)*cos(2*q[1])*(3 + cos(2*q[2])) + 4*dq[0]*dq[1]*sin(q[1])*sin(2*q[2]))*sin(2*q[3])) + 
            16*dq[2]*dq[3]*L4*cos(q[3])*cos(q[5])*sin(q[4]) + 4*pow(dq[0],2)*L2*cos(q[5])*sin(2*q[1])*sin(q[2])*sin(q[4]) + 4*pow(dq[0],2)*L4*cos(q[3])*cos(q[5])*sin(2*q[1])*sin(q[2])*sin(q[4]) - 8*dq[0]*dq[1]*L4*cos(q[5])*sin(q[1])*sin(q[3])*sin(q[4]) - 
            16*dq[0]*dq[3]*L4*cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3])*sin(q[4]) - 8*dq[0]*dq[1]*L4*pow(cos(q[2]),2)*cos(q[5])*sin(q[1])*sin(q[3])*sin(q[4]) + 16*dq[1]*dq[3]*L4*cos(q[5])*sin(q[2])*sin(q[3])*sin(q[4]) + 
            8*dq[0]*dq[1]*L4*cos(q[5])*sin(q[1])*pow(sin(q[2]),2)*sin(q[3])*sin(q[4]) - 2*pow(dq[0],2)*L4*cos(q[5])*sin(2*q[2])*sin(q[3])*sin(q[4]) + 4*pow(dq[1],2)*L4*cos(q[5])*sin(2*q[2])*sin(q[3])*sin(q[4]) - 
            2*pow(dq[0],2)*L4*cos(q[5])*pow(sin(q[1]),2)*sin(2*q[2])*sin(q[3])*sin(q[4]) + ((5*pow(dq[0],2) + 6*pow(dq[1],2) + 4*pow(dq[2],2) + 8*pow(dq[3],2))*L4 - (pow(dq[0],2) - 2*pow(dq[1],2))*L4*cos(2*q[2]) + 
                4*pow(dq[0],2)*L4*pow(cos(q[2]),2)*cos(2*q[3])*pow(sin(q[1]),2) + 8*L4*cos(q[2])*(2*dq[1]*dq[3] + dq[0]*sin(q[1])*(dq[1]*sin(q[2]) + dq[2]*sin(2*q[3]))) - 
                2*(2*L2*(-pow(dq[0],2) - 2*pow(dq[1],2) + pow(dq[0],2)*cos(2*q[1]))*cos(q[3]) + 2*L4*cos(2*q[3])*(pow(dq[2],2) - pow(dq[1],2)*pow(sin(q[2]),2) + dq[0]*dq[1]*sin(q[1])*sin(2*q[2])) + 
                L4*sin(q[2])*(-8*dq[0]*dq[3]*sin(q[1]) + pow(dq[0],2)*cos(2*q[1])*sin(q[2]) + 4*dq[1]*dq[2]*sin(2*q[3]))))*sin(q[5]) + 2*pow(dq[0],2)*L4*pow(cos(q[1]),2)*(cos(q[5])*sin(2*q[2])*sin(q[3])*sin(q[4]) - 2*cos(2*q[3])*sin(q[5])) + 
            8*cos(q[1])*(2*dq[0]*(dq[3]*L4*cos(q[3]) + dq[1]*cos(q[2])*(L2 + L4*cos(q[3])))*cos(q[5])*sin(q[4]) + dq[0]*sin(q[3])*(dq[0]*cos(q[2])*(L2 + 2*L4*cos(q[3]))*sin(q[1]) - 2*dq[1]*(L2 + L4*cos(q[3]))*sin(q[2]) + 2*dq[2]*L4*sin(q[3]))*sin(q[5]))))/8. + 
        (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*(-(dq[2]*dq[4]*cos(q[4])*pow(cos(q[5]),2)*sin(q[3])) - dq[0]*dq[4]*cos(q[1])*cos(q[4])*pow(cos(q[5]),2)*sin(q[3]) - 
        dq[1]*dq[2]*cos(q[4])*pow(cos(q[5]),2)*sin(q[2])*pow(sin(q[3]),2) - dq[0]*dq[1]*cos(q[1])*cos(q[4])*pow(cos(q[5]),2)*sin(q[2])*pow(sin(q[3]),2) + dq[3]*dq[4]*pow(cos(q[5]),2)*sin(q[4]) + dq[0]*dq[4]*pow(cos(q[5]),2)*sin(q[1])*sin(q[2])*sin(q[4]) + 
        dq[1]*dq[3]*pow(cos(q[5]),2)*sin(q[2])*sin(q[3])*sin(q[4]) + dq[0]*dq[1]*pow(cos(q[5]),2)*sin(q[1])*pow(sin(q[2]),2)*sin(q[3])*sin(q[4]) - pow(dq[4],2)*cos(q[5])*sin(q[5]) - 2*dq[1]*dq[4]*cos(q[5])*sin(q[2])*sin(q[3])*sin(q[5]) + 
        pow(dq[2],2)*pow(cos(q[4]),2)*cos(q[5])*pow(sin(q[3]),2)*sin(q[5]) + pow(dq[0],2)*pow(cos(q[1]),2)*pow(cos(q[4]),2)*cos(q[5])*pow(sin(q[3]),2)*sin(q[5]) - pow(dq[1],2)*cos(q[5])*pow(sin(q[2]),2)*pow(sin(q[3]),2)*sin(q[5]) - 
        2*dq[2]*dq[3]*cos(q[4])*cos(q[5])*sin(q[3])*sin(q[4])*sin(q[5]) - 2*dq[0]*dq[3]*cos(q[1])*cos(q[4])*cos(q[5])*sin(q[3])*sin(q[4])*sin(q[5]) - 2*dq[0]*dq[2]*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*sin(q[5]) - 
        2*pow(dq[0],2)*cos(q[1])*cos(q[4])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*sin(q[5]) + pow(dq[3],2)*cos(q[5])*pow(sin(q[4]),2)*sin(q[5]) + pow(dq[0],2)*cos(q[5])*pow(sin(q[1]),2)*pow(sin(q[2]),2)*pow(sin(q[4]),2)*sin(q[5]) + 
        dq[2]*dq[4]*cos(q[4])*sin(q[3])*pow(sin(q[5]),2) + dq[0]*dq[4]*cos(q[1])*cos(q[4])*sin(q[3])*pow(sin(q[5]),2) + dq[1]*dq[2]*cos(q[4])*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[5]),2) + dq[0]*dq[1]*cos(q[1])*cos(q[4])*sin(q[2])*pow(sin(q[3]),2)*pow(sin(q[5]),2) + 
        dq[0]*dq[2]*cos(q[1])*cos(q[4])*sin(2*q[3])*pow(sin(q[5]),2) - dq[3]*dq[4]*sin(q[4])*pow(sin(q[5]),2) - dq[0]*dq[4]*sin(q[1])*sin(q[2])*sin(q[4])*pow(sin(q[5]),2) - dq[1]*dq[3]*sin(q[2])*sin(q[3])*sin(q[4])*pow(sin(q[5]),2) - 
        dq[0]*dq[1]*sin(q[1])*pow(sin(q[2]),2)*sin(q[3])*sin(q[4])*pow(sin(q[5]),2) + pow(cos(q[2]),2)*(dq[0]*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[1]) - dq[1]*cos(q[5])*sin(q[4]) - dq[0]*sin(q[1])*sin(q[3])*sin(q[5]))*
            (dq[0]*cos(q[5])*sin(q[1])*sin(q[3]) + (dq[0]*cos(q[3])*cos(q[4])*sin(q[1]) - dq[1]*sin(q[4]))*sin(q[5])) + dq[0]*dq[2]*cos(q[1])*pow(cos(q[4]),2)*pow(sin(q[3]),2)*sin(2*q[5]) + dq[0]*dq[3]*sin(q[1])*sin(q[2])*pow(sin(q[4]),2)*sin(2*q[5]) + 
        (pow(cos(q[3]),2)*(2*dq[1]*cos(q[4])*sin(q[2])*((dq[2] + dq[0]*cos(q[1]))*cos(2*q[5]) + dq[1]*cos(q[4])*cos(q[5])*sin(q[2])*sin(q[5])) - pow(dq[2] + dq[0]*cos(q[1]),2)*sin(2*q[5])))/2. + 
        cos(q[3])*((dq[2] + dq[0]*cos(q[1]))*(pow(cos(q[5]),2)*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4]) - 2*cos(q[5])*(dq[4] + dq[1]*sin(q[2])*sin(q[3]))*sin(q[5]) - (dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4])*pow(sin(q[5]),2)) - 
            dq[1]*(dq[2] + dq[0]*cos(q[1]))*pow(cos(q[4]),2)*sin(q[2])*sin(q[3])*sin(2*q[5]) + cos(q[4])*(pow(cos(q[5]),2)*(-(pow(dq[2] + dq[0]*cos(q[1]),2)*sin(q[3])) + dq[1]*sin(q[2])*(dq[4] + dq[1]*sin(q[2])*sin(q[3]))) + 
                ((pow(dq[2],2) + pow(dq[0],2)*pow(cos(q[1]),2))*sin(q[3]) - dq[1]*sin(q[2])*(dq[4] + dq[1]*sin(q[2])*sin(q[3])))*pow(sin(q[5]),2) + dq[1]*sin(q[2])*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4])*sin(2*q[5]))) + 
        cos(q[2])*(cos(2*q[5])*(dq[1]*dq[4] + (pow(dq[1],2)*sin(q[2]) - dq[0]*sin(q[1])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])))*sin(q[3]))*sin(q[4]) + 
            (dq[0]*sin(q[1])*sin(q[3])*(dq[4] + dq[1]*sin(q[2])*sin(q[3])) + dq[1]*(dq[3] + dq[0]*sin(q[1])*sin(q[2]))*pow(sin(q[4]),2))*sin(2*q[5]) - dq[0]*pow(cos(q[3]),2)*cos(q[4])*sin(q[1])*((dq[2] + dq[0]*cos(q[1]))*cos(2*q[5]) + dq[1]*cos(q[4])*sin(q[2])*sin(2*q[5])) + 
            cos(q[4])*(dq[0]*sin(q[1])*((dq[2] + dq[0]*cos(q[1]))*cos(2*q[5])*pow(sin(q[3]),2) + dq[1]*sin(q[2])*sin(2*q[3])*pow(sin(q[5]),2)) - dq[1]*(dq[2] + dq[0]*cos(q[1]))*sin(q[3])*sin(q[4])*sin(2*q[5])) + 
            cos(q[3])*(-(dq[0]*dq[4]*cos(q[4])*cos(2*q[5])*sin(q[1])) - 2*dq[0]*dq[1]*cos(q[4])*pow(cos(q[5]),2)*sin(q[1])*sin(q[2])*sin(q[3]) + dq[1]*dq[2]*cos(2*q[5])*sin(q[4]) + dq[0]*dq[1]*cos(q[1])*cos(2*q[5])*sin(q[4]) + 
                (dq[0]*(dq[2] + dq[0]*cos(q[1]))*(1 + pow(cos(q[4]),2))*sin(q[1])*sin(q[3]) - cos(q[4])*(-(pow(dq[1],2)*sin(q[2])) + dq[0]*sin(q[1])*(dq[3] + dq[0]*sin(q[1])*sin(q[2])))*sin(q[4]))*sin(2*q[5])))));

    Coriolis_Comp(6,0)=((ia7zz + im7*kr7)*(cos(q[5])*(-((dq[0]*cos(q[1])*(dq[3] + dq[1]*cos(q[2]) + dq[5]*cos(q[4])) + dq[2]*(dq[3] - dq[1]*cos(q[2]) + dq[5]*cos(q[4]) - dq[0]*sin(q[1])*sin(q[2])))*sin(q[3])) + dq[5]*(dq[3] + dq[1]*cos(q[2]) + dq[0]*sin(q[1])*sin(q[2]))*sin(q[4])) + 
        (-(dq[4]*dq[5]) - dq[1]*dq[5]*sin(q[2])*sin(q[3]) + cos(q[4])*(dq[0]*sin(q[1])*(dq[4]*sin(q[2]) + dq[1]*sin(q[3])) + cos(q[2])*(dq[1]*dq[4] + dq[0]*dq[3]*sin(q[1])*sin(q[3])) + dq[3]*(dq[4] - dq[1]*sin(q[2])*sin(q[3]))) - dq[1]*dq[2]*sin(q[2])*sin(q[4]) + dq[0]*dq[1]*cos(q[1])*sin(q[2])*sin(q[4]) + 
            dq[2]*dq[4]*sin(q[3])*sin(q[4]) + dq[0]*dq[4]*cos(q[1])*sin(q[3])*sin(q[4]) + dq[0]*cos(q[2])*sin(q[1])*(dq[5]*sin(q[3]) + dq[2]*sin(q[4])))*sin(q[5]) - 
        cos(q[3])*(cos(q[5])*(dq[0]*(dq[1] + cos(q[2])*(dq[3] + dq[5]*cos(q[4])))*sin(q[1]) - dq[1]*(dq[3] + dq[5]*cos(q[4]))*sin(q[2])) + 
            (dq[2]*dq[5] + dq[0]*cos(q[1])*(dq[5] + (dq[3] + dq[1]*cos(q[2]))*cos(q[4])) + dq[2]*cos(q[4])*(dq[3] - dq[1]*cos(q[2]) - dq[0]*sin(q[1])*sin(q[2])) - dq[0]*dq[4]*cos(q[2])*sin(q[1])*sin(q[4]) + dq[1]*dq[4]*sin(q[2])*sin(q[4]))*sin(q[5]))));

    return Coriolis_Comp;
}

//***********************************Jacobian matrix****************************************
MatrixXd Jacobian_Matrix(double q[7])
{
    MatrixXd JacobComp(6,7);		//Jacobian computation matrix 6x7
    //Update Jacobian matrix
    //20190406 Results of Jacobian 6x7 - (correct)
    JacobComp(0,0) = - 1.0*d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + 1.0*cos(q[1])*cos(q[2])*sin(q[0])) + 1.0*cos(q[3])*sin(q[0])*sin(q[1])) - 1.0*d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 1.0*cos(q[1])*cos(q[2])*sin(q[0])) + 1.0*cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 1.0*cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d3*sin(q[0])*sin(q[1]);
    JacobComp(0,1) = 1.0*d5*(1.0*cos(q[0])*cos(q[1])*cos(q[3]) - 1.0*cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - 1.0*d7*(sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + 1.0*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(1.0*cos(q[0])*cos(q[1])*cos(q[3]) - 1.0*cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[0])*cos(q[1]);
    JacobComp(0,2) = 1.0*d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2]))) - 1.0*d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2]));
    JacobComp(0,3) = - 1.0*d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + 1.0*cos(q[0])*sin(q[1])*sin(q[3])) - 1.0*d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + 1.0*cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])));
    JacobComp(0,4) = 1.0*d7*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])));
    JacobComp(0,5) = 1.0*d7*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) - 1.0*cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))));
    JacobComp(0,6) = 0;

    JacobComp(1,0) = d3*cos(q[0])*sin(q[1]) - d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - d7*(sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 1.0*cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2]))) + cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])));
    JacobComp(1,1) = d5*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d7*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(1.0*cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - 1.0*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d3*cos(q[1])*sin(q[0]);
    JacobComp(1,2) = d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) - d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + 1.0*cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])));
    JacobComp(1,3) = d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 1.0*cos(q[3])*sin(q[0])*sin(q[1])));
    JacobComp(1,4) = -d7*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 1.0*sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 1.0*cos(q[1])*sin(q[0])*sin(q[2])));
    JacobComp(1,5) = d7*(cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 1.0*sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 1.0*cos(q[1])*sin(q[0])*sin(q[2]))) - sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])));
    JacobComp(1,6) = 0;

    JacobComp(2,0) = 0;
    JacobComp(2,1) = 1.0*d7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 1.0*cos(q[1])*sin(q[2])*sin(q[4])) - 1.0*cos(q[5])*(cos(q[3])*sin(q[1]) + 1.0*cos(q[1])*cos(q[2])*sin(q[3]))) - d3*sin(q[1]) - d5*(cos(q[3])*sin(q[1]) + 1.0*cos(q[1])*cos(q[2])*sin(q[3]));
    JacobComp(2,2) = 1.0*d7*(sin(q[5])*(1.0*cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + 1.0*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + 1.0*d5*sin(q[1])*sin(q[2])*sin(q[3]);
    JacobComp(2,3) = - d5*(cos(q[1])*sin(q[3]) + 1.0*cos(q[2])*cos(q[3])*sin(q[1])) - 1.0*d7*(1.0*cos(q[5])*(cos(q[1])*sin(q[3]) + 1.0*cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])));
    JacobComp(2,4) = 1.0*d7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 1.0*cos(q[4])*sin(q[1])*sin(q[2]));
    JacobComp(2,5) = -1.0*d7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 1.0*sin(q[1])*sin(q[2])*sin(q[4])) + 1.0*sin(q[5])*(cos(q[1])*cos(q[3]) - 1.0*cos(q[2])*sin(q[1])*sin(q[3])));
    JacobComp(2,6) = 0;

    JacobComp(3,0) = -1.0*sin(q[0]);
    JacobComp(3,1) = cos(q[0])*sin(q[1]);
    JacobComp(3,2) = - 1.0*cos(q[2])*sin(q[0]) - 1.0*cos(q[0])*cos(q[1])*sin(q[2]);
    JacobComp(3,3) = 1.0*cos(q[0])*cos(q[3])*sin(q[1]) - 1.0*sin(q[3])*(1.0*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2]));
    JacobComp(3,4) = sin(q[4])*(1.0*cos(q[3])*(1.0*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + 1.0*cos(q[0])*sin(q[1])*sin(q[3])) - 1.0*cos(q[4])*(1.0*cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2]));
    JacobComp(3,5) = - 1.0*cos(q[5])*(1.0*sin(q[3])*(1.0*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) - 1.0*cos(q[0])*cos(q[3])*sin(q[1])) - 1.0*sin(q[5])*(1.0*sin(q[4])*(1.0*cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2])) + 1.0*cos(q[4])*(1.0*cos(q[3])*(1.0*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + 1.0*cos(q[0])*sin(q[1])*sin(q[3])));
    JacobComp(3,6) = - 1.0*cos(q[5])*(1.0*sin(q[3])*(1.0*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) - 1.0*cos(q[0])*cos(q[3])*sin(q[1])) - 1.0*sin(q[5])*(1.0*sin(q[4])*(1.0*cos(q[2])*sin(q[0]) + 1.0*cos(q[0])*cos(q[1])*sin(q[2])) + 1.0*cos(q[4])*(1.0*cos(q[3])*(1.0*sin(q[0])*sin(q[2]) - 1.0*cos(q[0])*cos(q[1])*cos(q[2])) + 1.0*cos(q[0])*sin(q[1])*sin(q[3])));


    JacobComp(4,0) = cos(q[0]);
    JacobComp(4,1) = sin(q[0])*sin(q[1]);
    JacobComp(4,2) = cos(q[0])*cos(q[2]) - 1.0*cos(q[1])*sin(q[0])*sin(q[2]);
    JacobComp(4,3) = sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 1.0*cos(q[3])*sin(q[0])*sin(q[1]);
    JacobComp(4,4) = cos(q[4])*(cos(q[0])*cos(q[2]) - 1.0*cos(q[1])*sin(q[0])*sin(q[2])) - 1.0*sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 1.0*sin(q[0])*sin(q[1])*sin(q[3]));
    JacobComp(4,5) = sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 1.0*sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 1.0*cos(q[1])*sin(q[0])*sin(q[2]))) + cos(q[5])*(1.0*sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 1.0*cos(q[3])*sin(q[0])*sin(q[1]));
    JacobComp(4,6) = sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 1.0*sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 1.0*cos(q[1])*sin(q[0])*sin(q[2]))) + cos(q[5])*(1.0*sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 1.0*cos(q[3])*sin(q[0])*sin(q[1]));

    JacobComp(5,0) = 0;
    JacobComp(5,1) = 1.0*cos(q[1]);
    JacobComp(5,2) = 1.0*sin(q[1])*sin(q[2]);
    JacobComp(5,3) = 1.0*cos(q[1])*cos(q[3]) - 1.0*cos(q[2])*sin(q[1])*sin(q[3]);
    JacobComp(5,4) = sin(q[4])*(1.0*cos(q[1])*sin(q[3]) + 1.0*cos(q[2])*cos(q[3])*sin(q[1])) + 1.0*cos(q[4])*sin(q[1])*sin(q[2]);
    JacobComp(5,5) = cos(q[5])*(1.0*cos(q[1])*cos(q[3]) - 1.0*cos(q[2])*sin(q[1])*sin(q[3])) - 1.0*sin(q[5])*(1.0*cos(q[4])*(1.0*cos(q[1])*sin(q[3]) + 1.0*cos(q[2])*cos(q[3])*sin(q[1])) - 1.0*sin(q[1])*sin(q[2])*sin(q[4]));
    JacobComp(5,6) = cos(q[5])*(1.0*cos(q[1])*cos(q[3]) - 1.0*cos(q[2])*sin(q[1])*sin(q[3])) - 1.0*sin(q[5])*(1.0*cos(q[4])*(1.0*cos(q[1])*sin(q[3]) + 1.0*cos(q[2])*cos(q[3])*sin(q[1])) - 1.0*sin(q[1])*sin(q[2])*sin(q[4]));
    
    return JacobComp;
}

//***********************************Differentiate of Jacobian matrix****************************************
MatrixXd DiffJacobian_Matrix(double q[7], double dq[7])
{
    MatrixXd DiffJacobComp(6,7);		//Jacobian computation matrix 6x7

    DiffJacobComp(0,0)=-1.*d7*dq[4]*(cos(q[0])*(cos(q[2])*cos(q[4]) - 1.*cos(q[3])*sin(q[2])*sin(q[4])) + sin(q[0])*(sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*(-1.*cos(q[4])*sin(q[2]) - 1.*cos(q[2])*cos(q[3])*sin(q[4]))))*sin(q[5]) + 
							dq[3]*(sin(q[0])*sin(q[1])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[0])*sin(q[2])*(-1.*d5*cos(q[3]) - 1.*d7*cos(q[3])*cos(q[5]) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + 
								cos(q[1])*cos(q[2])*sin(q[0])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
							dq[2]*(cos(q[0])*(1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*d5*sin(q[3]) - 1.*d7*cos(q[5])*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
								cos(q[1])*sin(q[0])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
							dq[1]*sin(q[0])*(-1.*cos(q[1])*(d3 + cos(q[3])*(1.*d5 + 1.*d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) - 
							1.*d7*dq[5]*(sin(q[0])*sin(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[0])*(cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) + cos(q[2])*cos(q[5])*sin(q[4]) - sin(q[2])*sin(q[3])*sin(q[5])) - 
								1.*cos(q[1])*sin(q[0])*(cos(q[5])*sin(q[2])*sin(q[4]) + cos(q[2])*(-1.*cos(q[3])*cos(q[4])*cos(q[5]) + 1.*sin(q[3])*sin(q[5])))) + 
							dq[0]*(sin(q[0])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) - 
								1.*cos(q[0])*(sin(q[1])*(d3 + cos(q[3])*(1.*d5 + 1.*d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + cos(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))));

    DiffJacobComp(0,1)=1.*d7*dq[4]*cos(q[0])*(cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + 
                    dq[2]*cos(q[0])*sin(q[1])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) - 
                    1.*d7*dq[5]*cos(q[0])*(-(cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])) + cos(q[1])*(cos(q[4])*cos(q[5])*sin(q[3]) + 1.*cos(q[3])*sin(q[5])) + cos(q[2])*sin(q[1])*(1.*cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))) + 
                    dq[3]*cos(q[0])*(cos(q[1])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[1]*cos(q[0])*(sin(q[1])*(-1.*d3 + cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + cos(q[1])*(1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*d5*sin(q[3]) - 1.*d7*cos(q[5])*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[0]*sin(q[0])*(cos(q[1])*(-1.*d3 + cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))));

    DiffJacobComp(0,2)=1.*d7*dq[4]*(sin(q[0])*(cos(q[4])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[4])) - 1.*cos(q[0])*cos(q[1])*(cos(q[2])*cos(q[4]) - 1.*cos(q[3])*sin(q[2])*sin(q[4])))*sin(q[5]) + 
                    dq[3]*(cos(q[2])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*sin(q[2]))*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + 
                    1.*d7*dq[5]*(cos(q[5])*(-(cos(q[2])*(cos(q[3])*cos(q[4])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[4]))) + sin(q[2])*(-1.*cos(q[0])*cos(q[1])*cos(q[3])*cos(q[4]) + sin(q[0])*sin(q[4]))) + (cos(q[2])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*sin(q[2]))*sin(q[3])*sin(q[5])) + 
                    dq[1]*cos(q[0])*sin(q[1])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                    dq[2]*(cos(q[0])*cos(q[1])*(1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*d5*sin(q[3]) - 1.*d7*cos(q[5])*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                        sin(q[0])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[0]*(cos(q[0])*(1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*d5*sin(q[3]) - 1.*d7*cos(q[5])*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                        cos(q[1])*sin(q[0])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))));

    DiffJacobComp(0,3)=-1.*d7*dq[4]*(-(cos(q[0])*cos(q[3])*sin(q[1])) + (-1.*cos(q[0])*cos(q[1])*cos(q[2]) + sin(q[0])*sin(q[2]))*sin(q[3]))*sin(q[4])*sin(q[5]) + 
                    dq[2]*(cos(q[2])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*sin(q[2]))*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + 
                    dq[5]*(-1.*d7*cos(q[4])*cos(q[5])*(-1.*sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*(cos(q[3])*sin(q[1]) + 1.*cos(q[1])*cos(q[2])*sin(q[3]))) + 1.*d7*(cos(q[3])*sin(q[0])*sin(q[2]) + cos(q[0])*(-1.*cos(q[1])*cos(q[2])*cos(q[3]) + 1.*sin(q[1])*sin(q[3])))*sin(q[5])) + 
                    dq[0]*(sin(q[0])*sin(q[1])*(1.*d5*sin(q[3]) + 1.*d7*cos(q[5])*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[0])*sin(q[2])*(-1.*d5*cos(q[3]) - 1.*d7*cos(q[3])*cos(q[5]) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + 
                        cos(q[1])*cos(q[2])*sin(q[0])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[1]*cos(q[0])*(cos(q[1])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[3]*(sin(q[0])*sin(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[0])*
                        (cos(q[3])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[1]) - 1.*d7*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[5])) + sin(q[3])*(cos(q[1])*cos(q[2])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[1])*sin(q[5]))));

    DiffJacobComp(0,4)=1.*d7*dq[5]*cos(q[5])*(-(cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + (cos(q[3])*sin(q[0])*sin(q[2]) + cos(q[0])*(-1.*cos(q[1])*cos(q[2])*cos(q[3]) + sin(q[1])*sin(q[3])))*sin(q[4])) + 
                    1.*d7*dq[3]*(-1.*sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*(cos(q[3])*sin(q[1]) + 1.*cos(q[1])*cos(q[2])*sin(q[3])))*sin(q[4])*sin(q[5]) + 1.*d7*dq[1]*cos(q[0])*(cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + 
                    1.*d7*dq[2]*(sin(q[0])*(cos(q[4])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[4])) + cos(q[0])*cos(q[1])*(-1.*cos(q[2])*cos(q[4]) + 1.*cos(q[3])*sin(q[2])*sin(q[4])))*sin(q[5]) - 
                    1.*d7*dq[0]*(cos(q[0])*(cos(q[2])*cos(q[4]) - 1.*cos(q[3])*sin(q[2])*sin(q[4])) + sin(q[0])*(sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*(-1.*cos(q[4])*sin(q[2]) - 1.*cos(q[2])*cos(q[3])*sin(q[4]))))*sin(q[5]) + 
                    1.*d7*dq[4]*(sin(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4])) + cos(q[0])*(cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(-1.*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[4]))))*sin(q[5]);

    DiffJacobComp(0,5)=-1.*d7*dq[4]*cos(q[5])*(cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) - (cos(q[3])*sin(q[0])*sin(q[2]) + cos(q[0])*(-1.*cos(q[1])*cos(q[2])*cos(q[3]) + sin(q[1])*sin(q[3])))*sin(q[4])) + 
                        1.*d7*dq[2]*(-(cos(q[5])*(cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*sin(q[2])) + (cos(q[0])*cos(q[1])*cos(q[2]) - sin(q[0])*sin(q[2]))*sin(q[4]))) + (cos(q[2])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*sin(q[2]))*sin(q[3])*sin(q[5])) + 
                        1.*d7*dq[5]*(cos(q[5])*(sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*(-1.*cos(q[3])*sin(q[1]) - 1.*cos(q[1])*cos(q[2])*sin(q[3]))) + 
                            (sin(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4])) + cos(q[0])*(cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(-1.*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[4]))))*sin(q[5])) + 
                        1.*d7*dq[1]*cos(q[0])*(cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[2])*sin(q[1])*(-1.*cos(q[3])*cos(q[4])*cos(q[5]) + 1.*sin(q[3])*sin(q[5]))) + 
                        1.*d7*dq[3]*(sin(q[0])*sin(q[2])*(cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5])) - 1.*cos(q[0])*(cos(q[3])*(cos(q[4])*cos(q[5])*sin(q[1]) + 1.*cos(q[1])*cos(q[2])*sin(q[5])) + sin(q[3])*(1.*cos(q[1])*cos(q[2])*cos(q[4])*cos(q[5]) - 1.*sin(q[1])*sin(q[5])))) + 
                        1.*d7*dq[0]*(sin(q[0])*sin(q[1])*(cos(q[4])*cos(q[5])*sin(q[3]) + 1.*cos(q[3])*sin(q[5])) - cos(q[0])*(cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) + cos(q[2])*cos(q[5])*sin(q[4]) - sin(q[2])*sin(q[3])*sin(q[5])) + 
                            cos(q[1])*sin(q[0])*(cos(q[5])*sin(q[2])*sin(q[4]) + cos(q[2])*(-1.*cos(q[3])*cos(q[4])*cos(q[5]) + 1.*sin(q[3])*sin(q[5]))));

    DiffJacobComp(0,6)=0;

    //---------------------------------------------------------------------------------------------------------------
    DiffJacobComp(1,0)=-1.*d7*dq[4]*(-1.*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) + cos(q[2])*(cos(q[4])*sin(q[0]) + cos(q[0])*cos(q[1])*cos(q[3])*sin(q[4])) + cos(q[0])*(1.*cos(q[1])*cos(q[4])*sin(q[2]) - 1.*sin(q[1])*sin(q[3])*sin(q[4])))*sin(q[5]) + 
                    dq[5]*(-(d7*cos(q[5])*(sin(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4])) + cos(q[0])*(1.*cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(-1.*cos(q[2])*cos(q[3])*cos(q[4]) + 1.*sin(q[2])*sin(q[4]))))) + 
                        d7*(sin(q[0])*sin(q[2])*sin(q[3]) - cos(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))*sin(q[5])) + 
                    dq[2]*(sin(q[2])*(d7*sin(q[0])*sin(q[4])*sin(q[5]) + cos(q[0])*cos(q[1])*((d5 - d7*cos(q[5]))*sin(q[3]) - d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                        cos(q[2])*(-1.*d7*cos(q[0])*cos(q[1])*sin(q[4])*sin(q[5]) + sin(q[0])*((d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[1]*cos(q[0])*(cos(q[1])*(d3 + cos(q[3])*(d5 + 1.*d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) - 1.*sin(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[3]*(-(sin(q[0])*sin(q[2])*(cos(q[3])*(d5 + d7*cos(q[5])) - d7*cos(q[4])*sin(q[3])*sin(q[5]))) + cos(q[0])*(sin(q[1])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + 
                            cos(q[1])*cos(q[2])*(cos(q[3])*(d5 + d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])))) + dq[0]*(-(cos(q[0])*(d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5])))) - 
                        1.*sin(q[0])*(sin(q[1])*(d3 + cos(q[3])*(d5 + d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + cos(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5])))));

    DiffJacobComp(1,1)=d7*dq[4]*sin(q[0])*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (cos(q[2])*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + dq[2]*sin(q[0])*sin(q[1])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5]))) - 
                    1.*d7*dq[5]*sin(q[0])*(-1.*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[1])*(1.*cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))) + 
                    dq[3]*sin(q[0])*(cos(q[1])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + d7*cos(q[4])*sin(q[3])*sin(q[5]))) - 
                    1.*dq[1]*sin(q[0])*(sin(q[1])*(d3 + cos(q[3])*(d5 + d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + cos(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[0]*cos(q[0])*(cos(q[1])*(d3 + cos(q[3])*(d5 + d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) - 1.*sin(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))));

    DiffJacobComp(1,2)=-(d7*dq[4]*(cos(q[0])*(cos(q[4])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[4])) + cos(q[1])*sin(q[0])*(1.*cos(q[2])*cos(q[4]) - cos(q[3])*sin(q[2])*sin(q[4])))*sin(q[5])) + 
                    dq[3]*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))*(cos(q[3])*(d5 + d7*cos(q[5])) - d7*cos(q[4])*sin(q[3])*sin(q[5])) + dq[1]*sin(q[0])*sin(q[1])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                    dq[0]*(-1.*sin(q[2])*(-1.*d7*sin(q[0])*sin(q[4])*sin(q[5]) + cos(q[0])*cos(q[1])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5]))) - 
                        1.*cos(q[2])*(1.*d7*cos(q[0])*cos(q[1])*sin(q[4])*sin(q[5]) + sin(q[0])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[2]*(-1.*cos(q[1])*sin(q[0])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5]))) - 
                        1.*cos(q[0])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((d5 + d7*cos(q[5]))*sin(q[3]) + d7*cos(q[3])*cos(q[4])*sin(q[5])))) - 
                    d7*dq[5]*(cos(q[1])*sin(q[0])*(cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) + 1.*cos(q[2])*cos(q[5])*sin(q[4]) - 1.*sin(q[2])*sin(q[3])*sin(q[5])) + cos(q[0])*(cos(q[5])*sin(q[2])*sin(q[4]) + cos(q[2])*(-(cos(q[3])*cos(q[4])*cos(q[5])) + sin(q[3])*sin(q[5]))));

    DiffJacobComp(1,3)=d7*dq[4]*(1.*cos(q[3])*sin(q[0])*sin(q[1]) + (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))*sin(q[4])*sin(q[5]) + dq[2]*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))*(cos(q[3])*(d5 + d7*cos(q[5])) - d7*cos(q[4])*sin(q[3])*sin(q[5])) + 
                    d7*dq[5]*(-(cos(q[4])*cos(q[5])*(1.*cos(q[3])*sin(q[0])*sin(q[1]) + (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))) + (-(cos(q[3])*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[5])) + 
                    dq[1]*sin(q[0])*(cos(q[1])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[3]*(-1.*sin(q[3])*(cos(q[1])*cos(q[2])*(d5 + d7*cos(q[5]))*sin(q[0]) + cos(q[0])*(d5 + d7*cos(q[5]))*sin(q[2]) - 1.*d7*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[5])) - 
                        1.*cos(q[3])*(1.*d7*cos(q[0])*cos(q[4])*sin(q[2])*sin(q[5]) + sin(q[0])*((d5 + d7*cos(q[5]))*sin(q[1]) + 1.*d7*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[5])))) + 
                    dq[0]*(-(sin(q[0])*sin(q[2])*(cos(q[3])*(d5 + d7*cos(q[5])) - d7*cos(q[4])*sin(q[3])*sin(q[5]))) + cos(q[0])*(sin(q[1])*((-1.*d5 - 1.*d7*cos(q[5]))*sin(q[3]) - 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + 
                            cos(q[1])*cos(q[2])*(cos(q[3])*(d5 + d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))));

    DiffJacobComp(1,4)=-(d7*dq[5]*cos(q[5])*(cos(q[0])*(-(cos(q[2])*cos(q[4])) + cos(q[3])*sin(q[2])*sin(q[4])) + sin(q[0])*(-1.*sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*(1.*cos(q[4])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[4]))))) - 
                    d7*dq[3]*(-1.*cos(q[3])*sin(q[0])*sin(q[1]) - (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))*sin(q[4])*sin(q[5]) + 1.*d7*dq[1]*sin(q[0])*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) - 
                    d7*dq[4]*(-1.*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3]) + cos(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4])) + cos(q[1])*sin(q[0])*(cos(q[2])*cos(q[3])*cos(q[4]) - 1.*sin(q[2])*sin(q[4])))*sin(q[5]) - 
                    d7*dq[2]*(cos(q[0])*(cos(q[4])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[4])) + cos(q[1])*sin(q[0])*(1.*cos(q[2])*cos(q[4]) - cos(q[3])*sin(q[2])*sin(q[4])))*sin(q[5]) - 
                    1.*d7*dq[0]*(-1.*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) + cos(q[2])*(cos(q[4])*sin(q[0]) + cos(q[0])*cos(q[1])*cos(q[3])*sin(q[4])) + cos(q[0])*(1.*cos(q[1])*cos(q[4])*sin(q[2]) - 1.*sin(q[1])*sin(q[3])*sin(q[4])))*sin(q[5]);

    DiffJacobComp(1,5)=d7*dq[4]*cos(q[5])*(cos(q[4])*(cos(q[0])*cos(q[2]) - 1.*cos(q[1])*sin(q[0])*sin(q[2])) - 1.*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4])) + 
                    d7*dq[3]*(cos(q[4])*cos(q[5])*(-1.*cos(q[3])*sin(q[0])*sin(q[1]) - (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3])) + (-(cos(q[3])*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[5])) + 
                    d7*dq[0]*(cos(q[5])*(-(sin(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4]))) + cos(q[0])*(-1.*cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(cos(q[2])*cos(q[3])*cos(q[4]) - 1.*sin(q[2])*sin(q[4])))) + 
                        (sin(q[0])*sin(q[2])*sin(q[3]) - cos(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))*sin(q[5])) + 
                    d7*dq[5]*(-(cos(q[5])*(cos(q[3])*sin(q[0])*sin(q[1]) + (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))) - 
                        (-1.*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3]) + cos(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4])) + cos(q[1])*sin(q[0])*(cos(q[2])*cos(q[3])*cos(q[4]) - 1.*sin(q[2])*sin(q[4])))*sin(q[5])) - 
                    1.*d7*dq[1]*sin(q[0])*(-1.*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[1])*(1.*cos(q[4])*cos(q[5])*sin(q[3]) + cos(q[3])*sin(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))) + 
                    d7*dq[2]*(cos(q[1])*sin(q[0])*(-(cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2])) - 1.*cos(q[2])*cos(q[5])*sin(q[4]) + sin(q[2])*sin(q[3])*sin(q[5])) - cos(q[0])*(cos(q[5])*sin(q[2])*sin(q[4]) + cos(q[2])*(-(cos(q[3])*cos(q[4])*cos(q[5])) + sin(q[3])*sin(q[5]))));

    DiffJacobComp(1,6)=0;

    //------------------------------------------------------------------------------------------------------------------
    DiffJacobComp(2,0)=0;

    DiffJacobComp(2,1)=1.*d7*dq[4]*(-1.*sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*(1.*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*cos(q[3])*sin(q[4])))*sin(q[5]) + dq[2]*cos(q[1])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                    dq[3]*(sin(q[1])*((d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[1])*cos(q[2])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[1]*(-1.*cos(q[1])*(d3 + cos(q[3])*(d5 + 1.*d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    1.*d7*dq[5]*(sin(q[1])*(cos(q[4])*cos(q[5])*sin(q[3]) + 1.*cos(q[3])*sin(q[5])) - 1.*cos(q[1])*(-1.*cos(q[5])*sin(q[2])*sin(q[4]) + cos(q[2])*(cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))));

    DiffJacobComp(2,2)=1.*d7*dq[4]*sin(q[1])*(1.*cos(q[2])*cos(q[4]) - 1.*cos(q[3])*sin(q[2])*sin(q[4]))*sin(q[5]) + dq[3]*sin(q[1])*sin(q[2])*(cos(q[3])*(1.*d5 + 1.*d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) + 
                    1.*d7*dq[5]*sin(q[1])*(cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) + 1.*cos(q[2])*cos(q[5])*sin(q[4]) - 1.*sin(q[2])*sin(q[3])*sin(q[5])) + 
                    dq[2]*sin(q[1])*(-1.*d7*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5]))) + 
                    dq[1]*cos(q[1])*(1.*d7*cos(q[2])*sin(q[4])*sin(q[5]) + sin(q[2])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])));

    DiffJacobComp(2,3)=1.*d7*dq[4]*(cos(q[1])*cos(q[3]) - 1.*cos(q[2])*sin(q[1])*sin(q[3]))*sin(q[4])*sin(q[5]) + dq[2]*sin(q[1])*sin(q[2])*(cos(q[3])*(1.*d5 + 1.*d7*cos(q[5])) - 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])) - 
                    1.*d7*dq[5]*(-1.*cos(q[2])*sin(q[1])*(cos(q[4])*cos(q[5])*sin(q[3]) + 1.*cos(q[3])*sin(q[5])) + cos(q[1])*(cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))) + 
                    dq[3]*(cos(q[2])*sin(q[1])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[1])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[1]*(sin(q[1])*((1.*d5 + 1.*d7*cos(q[5]))*sin(q[3]) + 1.*d7*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[1])*cos(q[2])*(cos(q[3])*(-1.*d5 - 1.*d7*cos(q[5])) + 1.*d7*cos(q[4])*sin(q[3])*sin(q[5])));

    DiffJacobComp(2,4)=1.*d7*dq[5]*cos(q[5])*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (cos(q[2])*cos(q[3])*sin(q[1]) + cos(q[1])*sin(q[3]))*sin(q[4])) + 1.*d7*dq[3]*(cos(q[1])*cos(q[3]) - 1.*cos(q[2])*sin(q[1])*sin(q[3]))*sin(q[4])*sin(q[5]) + 
                    1.*d7*dq[2]*sin(q[1])*(1.*cos(q[2])*cos(q[4]) - 1.*cos(q[3])*sin(q[2])*sin(q[4]))*sin(q[5]) + 1.*d7*dq[4]*(cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1]) + cos(q[1])*cos(q[4])*sin(q[3]) - 1.*sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 
                    1.*d7*dq[1]*(-(sin(q[1])*sin(q[3])*sin(q[4])) + cos(q[1])*(1.*cos(q[4])*sin(q[2]) + cos(q[2])*cos(q[3])*sin(q[4])))*sin(q[5]);

    DiffJacobComp(2,5)=-1.*d7*dq[4]*cos(q[5])*(-1.*cos(q[4])*sin(q[1])*sin(q[2]) - (cos(q[2])*cos(q[3])*sin(q[1]) + cos(q[1])*sin(q[3]))*sin(q[4])) + 1.*d7*dq[2]*sin(q[1])*(cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) + 1.*cos(q[2])*cos(q[5])*sin(q[4]) - 1.*sin(q[2])*sin(q[3])*sin(q[5])) + 
                    dq[5]*(-1.*d7*cos(q[5])*(cos(q[1])*cos(q[3]) - 1.*cos(q[2])*sin(q[1])*sin(q[3])) + 1.*d7*(cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1]) + cos(q[1])*cos(q[4])*sin(q[3]) - 1.*sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5])) - 
                    1.*d7*dq[3]*(cos(q[2])*sin(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[1])*(cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))) - 
                    1.*d7*dq[1]*(sin(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[1])*(-1.*cos(q[5])*sin(q[2])*sin(q[4]) + cos(q[2])*(cos(q[3])*cos(q[4])*cos(q[5]) - 1.*sin(q[3])*sin(q[5]))));

    DiffJacobComp(2,6)=0;

    //-------------------------------------------------------------------------------------------------------------------
    DiffJacobComp(3,0)=-1.*dq[0]*cos(q[0]);
    
    DiffJacobComp(3,1)=dq[1]*cos(q[0])*cos(q[1]) - dq[0]*sin(q[0])*sin(q[1]);
   
    DiffJacobComp(3,2)=1.*dq[1]*cos(q[0])*sin(q[1])*sin(q[2]) + dq[2]*(-1.*cos(q[0])*cos(q[1])*cos(q[2]) + 1.*sin(q[0])*sin(q[2])) + dq[0]*(-1.*cos(q[0])*cos(q[2]) + 1.*cos(q[1])*sin(q[0])*sin(q[2]));

    DiffJacobComp(3,3)=dq[2]*(-1.*cos(q[2])*sin(q[0]) - 1.*cos(q[0])*cos(q[1])*sin(q[2]))*sin(q[3]) + dq[1]*cos(q[0])*(1.*cos(q[1])*cos(q[3]) - 1.*cos(q[2])*sin(q[1])*sin(q[3])) + dq[0]*(-1.*cos(q[3])*sin(q[0])*sin(q[1]) + (-1.*cos(q[1])*cos(q[2])*sin(q[0]) - 1.*cos(q[0])*sin(q[2]))*sin(q[3])) + 
                    dq[3]*(-1.*cos(q[3])*sin(q[0])*sin(q[2]) + cos(q[0])*(1.*cos(q[1])*cos(q[2])*cos(q[3]) - 1.*sin(q[1])*sin(q[3])));

    DiffJacobComp(3,4)=1.*dq[3]*(-1.*sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*(1.*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*cos(q[2])*sin(q[3])))*sin(q[4]) + 1.*dq[1]*cos(q[0])*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*sin(q[3]))*sin(q[4])) + 
                    dq[2]*(sin(q[0])*(1.*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*cos(q[3])*sin(q[4])) + cos(q[0])*cos(q[1])*(-1.*cos(q[2])*cos(q[4]) + 1.*cos(q[3])*sin(q[2])*sin(q[4]))) + 
                    dq[0]*(cos(q[0])*(-1.*cos(q[2])*cos(q[4]) + 1.*cos(q[3])*sin(q[2])*sin(q[4])) + sin(q[0])*(-1.*sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*(1.*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*cos(q[3])*sin(q[4])))) + 
                    dq[4]*(sin(q[0])*(1.*cos(q[3])*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*sin(q[4])) + cos(q[0])*(1.*cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(-1.*cos(q[2])*cos(q[3])*cos(q[4]) + 1.*sin(q[2])*sin(q[4]))));

    DiffJacobComp(3,5)=-1.*dq[4]*(-1.*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) + cos(q[2])*(1.*cos(q[4])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[4])) + cos(q[0])*(1.*cos(q[1])*cos(q[4])*sin(q[2]) - 1.*sin(q[1])*sin(q[3])*sin(q[4])))*sin(q[5]) + 
                    dq[0]*(-1.*cos(q[0])*cos(q[5])*sin(q[2])*sin(q[3]) + 1.*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5]) - 1.*cos(q[0])*cos(q[2])*sin(q[4])*sin(q[5]) + 
                        cos(q[3])*(-1.*cos(q[5])*sin(q[0])*sin(q[1]) + cos(q[4])*(-1.*cos(q[1])*cos(q[2])*sin(q[0]) - 1.*cos(q[0])*sin(q[2]))*sin(q[5])) + cos(q[1])*sin(q[0])*(-1.*cos(q[2])*cos(q[5])*sin(q[3]) + 1.*sin(q[2])*sin(q[4])*sin(q[5]))) + 
                    dq[2]*(cos(q[2])*(-1.*cos(q[5])*sin(q[0])*sin(q[3]) + (-1.*cos(q[3])*cos(q[4])*sin(q[0]) - 1.*cos(q[0])*cos(q[1])*sin(q[4]))*sin(q[5])) + sin(q[2])*(1.*sin(q[0])*sin(q[4])*sin(q[5]) + cos(q[0])*cos(q[1])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[1]*cos(q[0])*(cos(q[1])*(1.*cos(q[3])*cos(q[5]) - 1.*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(1.*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[5]*(sin(q[0])*(-1.*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) - 1.*cos(q[2])*cos(q[5])*sin(q[4]) + 1.*sin(q[2])*sin(q[3])*sin(q[5])) + 
                        cos(q[0])*(sin(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[1])*(1.*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5]) - 1.*cos(q[5])*sin(q[2])*sin(q[4]) - 1.*cos(q[2])*sin(q[3])*sin(q[5])))) + 
                    dq[3]*(sin(q[0])*sin(q[2])*(-1.*cos(q[3])*cos(q[5]) + 1.*cos(q[4])*sin(q[3])*sin(q[5])) + cos(q[0])*(sin(q[1])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[1])*cos(q[2])*(1.*cos(q[3])*cos(q[5]) - 1.*cos(q[4])*sin(q[3])*sin(q[5]))));

    DiffJacobComp(3,6)=-1.*dq[4]*(-1.*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) + cos(q[2])*(1.*cos(q[4])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[4])) + cos(q[0])*(1.*cos(q[1])*cos(q[4])*sin(q[2]) - 1.*sin(q[1])*sin(q[3])*sin(q[4])))*sin(q[5]) + 
                    dq[0]*(-1.*cos(q[0])*cos(q[5])*sin(q[2])*sin(q[3]) + 1.*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[3])*sin(q[5]) - 1.*cos(q[0])*cos(q[2])*sin(q[4])*sin(q[5]) + 
                        cos(q[3])*(-1.*cos(q[5])*sin(q[0])*sin(q[1]) + cos(q[4])*(-1.*cos(q[1])*cos(q[2])*sin(q[0]) - 1.*cos(q[0])*sin(q[2]))*sin(q[5])) + cos(q[1])*sin(q[0])*(-1.*cos(q[2])*cos(q[5])*sin(q[3]) + 1.*sin(q[2])*sin(q[4])*sin(q[5]))) + 
                    dq[2]*(cos(q[2])*(-1.*cos(q[5])*sin(q[0])*sin(q[3]) + (-1.*cos(q[3])*cos(q[4])*sin(q[0]) - 1.*cos(q[0])*cos(q[1])*sin(q[4]))*sin(q[5])) + sin(q[2])*(1.*sin(q[0])*sin(q[4])*sin(q[5]) + cos(q[0])*cos(q[1])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[1]*cos(q[0])*(cos(q[1])*(1.*cos(q[3])*cos(q[5]) - 1.*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(1.*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[5]*(sin(q[0])*(-1.*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]) - 1.*cos(q[2])*cos(q[5])*sin(q[4]) + 1.*sin(q[2])*sin(q[3])*sin(q[5])) + 
                        cos(q[0])*(sin(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[1])*(1.*cos(q[2])*cos(q[3])*cos(q[4])*cos(q[5]) - 1.*cos(q[5])*sin(q[2])*sin(q[4]) - 1.*cos(q[2])*sin(q[3])*sin(q[5])))) + 
                    dq[3]*(sin(q[0])*sin(q[2])*(-1.*cos(q[3])*cos(q[5]) + 1.*cos(q[4])*sin(q[3])*sin(q[5])) + cos(q[0])*(sin(q[1])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[1])*cos(q[2])*(1.*cos(q[3])*cos(q[5]) - 1.*cos(q[4])*sin(q[3])*sin(q[5]))));

    //-----------------------------------------------------------------------------------------------------------------
    DiffJacobComp(4,0)=-(dq[0]*sin(q[0]));
    DiffJacobComp(4,1)=dq[1]*cos(q[1])*sin(q[0]) + dq[0]*cos(q[0])*sin(q[1]);
    DiffJacobComp(4,2)=1.*dq[1]*sin(q[0])*sin(q[1])*sin(q[2]) + dq[2]*(-1.*cos(q[1])*cos(q[2])*sin(q[0]) - cos(q[0])*sin(q[2])) + dq[0]*(-(cos(q[2])*sin(q[0])) - 1.*cos(q[0])*cos(q[1])*sin(q[2]));

    DiffJacobComp(4,3)=dq[2]*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))*sin(q[3]) + dq[1]*sin(q[0])*(1.*cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + dq[3]*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3])) + 
                    dq[0]*(-(sin(q[0])*sin(q[2])*sin(q[3])) + cos(q[0])*(1.*cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])));

    DiffJacobComp(4,4)=dq[3]*(1.*cos(q[3])*sin(q[0])*sin(q[1]) + 1.*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))*sin(q[4]) + 
                    dq[4]*(-1.*cos(q[4])*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3])) - 1.*(cos(q[0])*cos(q[2]) - 1.*cos(q[1])*sin(q[0])*sin(q[2]))*sin(q[4])) + 
                    dq[1]*sin(q[0])*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*sin(q[3]))*sin(q[4])) + 
                    dq[0]*(-1.*cos(q[4])*(cos(q[2])*sin(q[0]) + 1.*cos(q[0])*cos(q[1])*sin(q[2])) - 1.*(-1.*cos(q[3])*sin(q[0])*sin(q[2]) + cos(q[0])*(cos(q[1])*cos(q[2])*cos(q[3]) - 1.*sin(q[1])*sin(q[3])))*sin(q[4])) + 
                    dq[2]*(-1.*cos(q[0])*(cos(q[4])*sin(q[2]) + 1.*cos(q[2])*cos(q[3])*sin(q[4])) + cos(q[1])*sin(q[0])*(-1.*cos(q[2])*cos(q[4]) + 1.*cos(q[3])*sin(q[2])*sin(q[4])));

    DiffJacobComp(4,5)=dq[4]*(cos(q[4])*(cos(q[0])*cos(q[2]) - 1.*cos(q[1])*sin(q[0])*sin(q[2])) - 1.*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + 
                    dq[5]*(cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[5])*(cos(q[0])*cos(q[2]) - 1.*cos(q[1])*sin(q[0])*sin(q[2]))*sin(q[4]) - 1.*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[5]) - 
                        1.*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3])*sin(q[5])) + dq[3]*(cos(q[5])*(1.*cos(q[3])*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2])) - 1.*sin(q[0])*sin(q[1])*sin(q[3])) + 
                        cos(q[4])*(-1.*cos(q[3])*sin(q[0])*sin(q[1]) - (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))*sin(q[5])) + 
                    dq[0]*(cos(q[5])*(-1.*sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*(1.*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*cos(q[2])*sin(q[3]))) + 
                        (-(sin(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4]))) + cos(q[0])*(-1.*cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(cos(q[2])*cos(q[3])*cos(q[4]) - 1.*sin(q[2])*sin(q[4]))))*sin(q[5])) + 
                    dq[1]*sin(q[0])*(cos(q[1])*(1.*cos(q[3])*cos(q[5]) - 1.*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(1.*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[2]*(cos(q[1])*sin(q[0])*(-1.*cos(q[5])*sin(q[2])*sin(q[3]) - 1.*(cos(q[3])*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*sin(q[4]))*sin(q[5])) + cos(q[0])*(-(sin(q[2])*sin(q[4])*sin(q[5])) + cos(q[2])*(1.*cos(q[5])*sin(q[3]) + cos(q[3])*cos(q[4])*sin(q[5]))));

    DiffJacobComp(4,6)=dq[4]*(cos(q[4])*(cos(q[0])*cos(q[2]) - 1.*cos(q[1])*sin(q[0])*sin(q[2])) - 1.*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + 
                    dq[5]*(cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[2])*cos(q[3])*sin(q[0]) + cos(q[0])*cos(q[3])*sin(q[2]) - 1.*sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[5])*(cos(q[0])*cos(q[2]) - 1.*cos(q[1])*sin(q[0])*sin(q[2]))*sin(q[4]) - 1.*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[5]) - 
                        1.*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3])*sin(q[5])) + dq[3]*(cos(q[5])*(1.*cos(q[3])*(cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2])) - 1.*sin(q[0])*sin(q[1])*sin(q[3])) + 
                        cos(q[4])*(-1.*cos(q[3])*sin(q[0])*sin(q[1]) - (cos(q[1])*cos(q[2])*sin(q[0]) + cos(q[0])*sin(q[2]))*sin(q[3]))*sin(q[5])) + 
                    dq[0]*(cos(q[5])*(-1.*sin(q[0])*sin(q[2])*sin(q[3]) + cos(q[0])*(1.*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*cos(q[2])*sin(q[3]))) + 
                        (-(sin(q[0])*(cos(q[3])*cos(q[4])*sin(q[2]) + cos(q[2])*sin(q[4]))) + cos(q[0])*(-1.*cos(q[4])*sin(q[1])*sin(q[3]) + cos(q[1])*(cos(q[2])*cos(q[3])*cos(q[4]) - 1.*sin(q[2])*sin(q[4]))))*sin(q[5])) + 
                    dq[1]*sin(q[0])*(cos(q[1])*(1.*cos(q[3])*cos(q[5]) - 1.*cos(q[4])*sin(q[3])*sin(q[5])) + sin(q[1])*(1.*sin(q[2])*sin(q[4])*sin(q[5]) + cos(q[2])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])))) + 
                    dq[2]*(cos(q[1])*sin(q[0])*(-1.*cos(q[5])*sin(q[2])*sin(q[3]) - 1.*(cos(q[3])*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*sin(q[4]))*sin(q[5])) + cos(q[0])*(-(sin(q[2])*sin(q[4])*sin(q[5])) + cos(q[2])*(1.*cos(q[5])*sin(q[3]) + cos(q[3])*cos(q[4])*sin(q[5]))));

    //------------------------------------------------------------------------------------------------------------------
    DiffJacobComp(5,0)=0;
    DiffJacobComp(5,1)=-1.*dq[1]*sin(q[1]);
    DiffJacobComp(5,2)=1.*dq[2]*cos(q[2])*sin(q[1]) + 1.*dq[1]*cos(q[1])*sin(q[2]);
    DiffJacobComp(5,3)=1.*dq[2]*sin(q[1])*sin(q[2])*sin(q[3]) + dq[3]*(-1.*cos(q[2])*cos(q[3])*sin(q[1]) - 1.*cos(q[1])*sin(q[3])) + dq[1]*(-1.*cos(q[3])*sin(q[1]) - 1.*cos(q[1])*cos(q[2])*sin(q[3]));

    DiffJacobComp(5,4)=dq[3]*(1.*cos(q[1])*cos(q[3]) - 1.*cos(q[2])*sin(q[1])*sin(q[3]))*sin(q[4]) + dq[2]*sin(q[1])*(1.*cos(q[2])*cos(q[4]) - 1.*cos(q[3])*sin(q[2])*sin(q[4])) + dq[4]*(1.*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1]) + 1.*cos(q[1])*cos(q[4])*sin(q[3]) - 1.*sin(q[1])*sin(q[2])*sin(q[4])) + 
                    dq[1]*(-1.*sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*(1.*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*cos(q[3])*sin(q[4])));

    DiffJacobComp(5,5)=1.*dq[4]*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + dq[2]*sin(q[1])*(1.*cos(q[5])*sin(q[2])*sin(q[3]) + (1.*cos(q[3])*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*sin(q[4]))*sin(q[5])) + 
                    dq[5]*(1.*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[2])*sin(q[1])*(-1.*cos(q[3])*cos(q[4])*cos(q[5]) + 1.*sin(q[3])*sin(q[5]))) + 
                    dq[3]*(cos(q[1])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[2])*sin(q[1])*(-1.*cos(q[3])*cos(q[5]) + 1.*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[1]*(1.*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) + cos(q[3])*(-1.*cos(q[5])*sin(q[1]) - 1.*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[5])) + cos(q[1])*(-1.*cos(q[2])*cos(q[5])*sin(q[3]) + 1.*sin(q[2])*sin(q[4])*sin(q[5])));

    DiffJacobComp(5,6)=1.*dq[4]*(1.*cos(q[4])*sin(q[1])*sin(q[2]) + (1.*cos(q[2])*cos(q[3])*sin(q[1]) + 1.*cos(q[1])*sin(q[3]))*sin(q[4]))*sin(q[5]) + dq[2]*sin(q[1])*(1.*cos(q[5])*sin(q[2])*sin(q[3]) + (1.*cos(q[3])*cos(q[4])*sin(q[2]) + 1.*cos(q[2])*sin(q[4]))*sin(q[5])) + 
                    dq[5]*(1.*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[1])*(-1.*cos(q[4])*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*sin(q[5])) + cos(q[2])*sin(q[1])*(-1.*cos(q[3])*cos(q[4])*cos(q[5]) + 1.*sin(q[3])*sin(q[5]))) + 
                    dq[3]*(cos(q[1])*(-1.*cos(q[5])*sin(q[3]) - 1.*cos(q[3])*cos(q[4])*sin(q[5])) + cos(q[2])*sin(q[1])*(-1.*cos(q[3])*cos(q[5]) + 1.*cos(q[4])*sin(q[3])*sin(q[5]))) + 
                    dq[1]*(1.*cos(q[4])*sin(q[1])*sin(q[3])*sin(q[5]) + cos(q[3])*(-1.*cos(q[5])*sin(q[1]) - 1.*cos(q[1])*cos(q[2])*cos(q[4])*sin(q[5])) + cos(q[1])*(-1.*cos(q[2])*cos(q[5])*sin(q[3]) + 1.*sin(q[2])*sin(q[4])*sin(q[5])));

    return DiffJacobComp;
}


//***********************************Torque minimization function: CASE1 Gravity + TauE****************************************
//% dhdq(1,1) = simplify(transpose(diff((G+TauE),q1))*K*(G+TauE));
MatrixXd TorqueOptCase1(double q[7])
{
    MatrixXd dhdq(7,1);		//Gradient of cost funtion

     
    dhdq(0,0) = K4*(fy*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + fx*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) 
                + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fbet*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + falp*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])))*(fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) 
                - fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) 
                + fz*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + 
                falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + 
                g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) - K3*(falp*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + 
                fbet*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) - fx*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) 
                - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - 
                falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) 
                - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*sin(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*sin(q[1])*sin(q[2])) - K2*(fy*(d3*cos(q[0])*cos(q[1]) - 
                d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - fx*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fbet*cos(q[0])*sin(q[1]) - 
                falp*sin(q[0])*sin(q[1]))*(fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - 
                cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) - fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) - fgam*cos(q[1]) - 
                fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - 
                lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) - falp*cos(q[0])*sin(q[1]) - fbet*sin(q[0])*sin(q[1]) + g_acc*lg3*m3*sin(q[1]) + 
                g_acc*lg4*m4*sin(q[1])) + K7*(fbet*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + falp*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))) + 
                K6*(fbet*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + falp*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                d7*fy*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + 
                K1*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0]))*(fy*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - fx*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + falp*cos(q[0]) + fbet*sin(q[0])) + K5*(falp*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fbet*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) 
                - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])));

    dhdq(1,0) = K3*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - falp*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) 
                + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*sin(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*sin(q[1])*sin(q[2]))*(fx*(d7*(sin(q[5])*(cos(q[0])*cos(q[2])*sin(q[1])*sin(q[4]) + 
                cos(q[0])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[0])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + fy*(d7*(sin(q[5])*(cos(q[2])*sin(q[0])*sin(q[1])*sin(q[4]) + 
                cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + fz*(d7*(sin(q[5])*(cos(q[1])*cos(q[2])*sin(q[4]) + 
                cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])) + cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])) + d5*cos(q[1])*sin(q[2])*sin(q[3])) + g_acc*cos(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) 
                + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*cos(q[1])*sin(q[2]) + falp*cos(q[0])*sin(q[1])*sin(q[2]) + 
                fbet*sin(q[0])*sin(q[1])*sin(q[2])) - K4*(fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) - fgam*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))*(fz*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + d7*(cos(q[5])*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) - fx*(d7*(cos(q[5])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3]))) + d5*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1]))) - fgam*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])) + falp*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - fy*(d7*(cos(q[5])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) + 
                cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3]))) + d5*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1]))) + fbet*(cos(q[1])*cos(q[3])*sin(q[0]) - 
                cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + g_acc*m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) 
                + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + g_acc*lg6*m6*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]))) + K5*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2])))*(falp*(sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[0])*cos(q[4])*sin(q[1])*sin(q[2])) + fbet*(sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) + cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])) - fgam*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2])) - 
                d7*fz*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2])) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[0])*cos(q[4])*sin(q[1])*sin(q[2])) + d7*fy*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) + cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])) - 
                g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2]))) + K2*(fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - 
                d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) - 
                fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) - fgam*cos(q[1]) - fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + 
                d3*sin(q[1])) - falp*cos(q[0])*sin(q[1]) - fbet*sin(q[0])*sin(q[1]) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1]))*(fx*(d3*cos(q[0])*sin(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[0])*cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d5*cos(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + 
                fy*(d5*sin(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - 
                cos(q[5])*sin(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[0])*sin(q[1])) + fgam*sin(q[1]) + fz*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                d7*(sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])) + 
                g_acc*m5*(lg5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) + g_acc*m6*(lg6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) + g_acc*m7*(d5*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - lg7*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d3*cos(q[1])) - falp*cos(q[0])*cos(q[1]) - fbet*cos(q[1])*sin(q[0]) + g_acc*lg3*m3*cos(q[1]) + g_acc*lg4*m4*cos(q[1])) - K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(falp*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - 
                sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + fbet*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - 
                cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + 
                fgam*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + 
                K1*(fx*(d7*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - 
                sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) - fy*(d7*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - 
                cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[0])*cos(q[1])*cos(q[3]) - 
                cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[0])*cos(q[1])))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0])) - 
                K6*(falp*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - 
                cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + fbet*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + fgam*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d7*fz*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3]))) - d7*fx*(sin(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + cos(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) 
                - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) - d7*fy*(sin(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))));

    dhdq(2,0) = K5*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])))*(falp*(cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[3])*sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*(cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])) + d7*fz*sin(q[5])*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])) + 
                d7*fx*sin(q[5])*(cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[3])*sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(cos(q[4])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*lg7*m7*sin(q[5])*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4]))) - 
                K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(falp*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + 
                cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3]))) - K4*(fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) - fgam*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))*(fy*(d7*(cos(q[3])*cos(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) - 
                cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + d5*cos(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - fx*(d7*(cos(q[3])*cos(q[5])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])) - cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d5*cos(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                fz*(d7*(cos(q[3])*cos(q[5])*sin(q[1])*sin(q[2]) - cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])) + d5*cos(q[3])*sin(q[1])*sin(q[2])) - falp*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + 
                fbet*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + g_acc*m7*(lg7*(cos(q[3])*cos(q[5])*sin(q[1])*sin(q[2]) - cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])) + d5*cos(q[3])*sin(q[1])*sin(q[2])) + 
                fgam*sin(q[1])*sin(q[2])*sin(q[3]) + g_acc*lg5*m5*cos(q[3])*sin(q[1])*sin(q[2]) + g_acc*lg6*m6*cos(q[3])*sin(q[1])*sin(q[2])) - K6*(falp*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                fgam*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d7*fx*(cos(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                d7*fz*(cos(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])) + g_acc*lg7*m7*(cos(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + 
                cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])))) - K1*(fx*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fy*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) 
                - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0])) + K3*(falp*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - fbet*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                fx*(d7*(sin(q[5])*(sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + cos(q[3])*cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) + cos(q[5])*sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2]))) + d5*sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + 
                cos(q[3])*cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0]))) + cos(q[5])*sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0]))) + d5*sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0]))) 
                - fz*(d7*(sin(q[5])*(sin(q[1])*sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3])) - d5*cos(q[2])*sin(q[1])*sin(q[3])) + g_acc*sin(q[1])*(d5*m7*cos(q[2])*sin(q[3]) + 
                lg5*m5*cos(q[2])*sin(q[3]) + lg6*m6*cos(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*cos(q[5])*sin(q[3]) - lg7*m7*sin(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5])) + 
                fgam*cos(q[2])*sin(q[1]))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - 
                falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) 
                - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*sin(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*sin(q[1])*sin(q[2])) - 
                K2*(fx*(d7*(cos(q[0])*sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[0])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                fy*(d7*(sin(q[0])*sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                fz*(d7*(sin(q[5])*(cos(q[1])*cos(q[2])*sin(q[4]) + cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])) + cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])) + d5*cos(q[1])*sin(q[2])*sin(q[3])) + 
                g_acc*m7*(lg7*(sin(q[5])*(cos(q[1])*cos(q[2])*sin(q[4]) + cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])) + cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])) + d5*cos(q[1])*sin(q[2])*sin(q[3])) + g_acc*lg5*m5*cos(q[1])*sin(q[2])*sin(q[3]) + 
                g_acc*lg6*m6*cos(q[1])*sin(q[2])*sin(q[3]))*(fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) - fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d3*cos(q[1])*sin(q[0])) - fgam*cos(q[1]) - fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + 
                d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + 
                g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) - falp*cos(q[0])*sin(q[1]) - 
                fbet*sin(q[0])*sin(q[1]) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1]));

    dhdq(3,0) = K4*(fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) - fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fz*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) - fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))*(fgam*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - fx*(d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1]))) + fy*(d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + fz*(d5*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) + d7*(cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))) + falp*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - fbet*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + g_acc*m7*(d5*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) + lg7*(cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))) + g_acc*lg5*m5*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) + g_acc*lg6*m6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) - fbet*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])))) - K6*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))))*(fbet*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) - falp*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) - fgam*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d7*fz*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + d7*fx*(sin(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) - d7*fy*(sin(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + 
                cos(q[4])*cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + g_acc*lg7*m7*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - K2*(fz*(d7*(sin(q[5])*(cos(q[3])*cos(q[4])*sin(q[1]) + cos(q[1])*cos(q[2])*cos(q[4])*sin(q[3])) + cos(q[5])*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3]))) + d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]))) - fy*(d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[3])*cos(q[4]) - cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])) + 
                cos(q[5])*sin(q[0])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) + d5*sin(q[0])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) - fx*(d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[3])*cos(q[4]) - 
                cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])) + cos(q[0])*cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) + d5*cos(q[0])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) + g_acc*m7*(d5*(sin(q[1])*sin(q[3]) 
                - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*lg5*m5*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])) + g_acc*lg6*m6*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])))*(fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) - fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) - fgam*cos(q[1]) - fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*m7*(d5*(cos(q[3])*sin(q[1]) 
                + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) - 
                falp*cos(q[0])*sin(q[1]) - fbet*sin(q[0])*sin(q[1]) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1])) + K3*(fy*(d7*(cos(q[3])*cos(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) - 
                cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + d5*cos(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - fx*(d7*(cos(q[3])*cos(q[5])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])) - cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d5*cos(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                fz*(d7*(cos(q[3])*cos(q[5])*sin(q[1])*sin(q[2]) - cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])) + d5*cos(q[3])*sin(q[1])*sin(q[2])) + g_acc*sin(q[1])*(d5*m7*cos(q[3])*sin(q[2]) + lg5*m5*cos(q[3])*sin(q[2]) + 
                lg6*m6*cos(q[3])*sin(q[2]) + lg7*m7*cos(q[3])*cos(q[5])*sin(q[2]) - lg7*m7*cos(q[4])*sin(q[2])*sin(q[3])*sin(q[5])))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + 
                cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + 
                fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) 
                + g_acc*sin(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + 
                lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*sin(q[1])*sin(q[2])) + K5*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - 
                fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2])))*(fgam*sin(q[4])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - falp*sin(q[4])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                fbet*sin(q[4])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*fz*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                d7*fx*sin(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*fy*sin(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*lg7*m7*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + K1*(fy*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                + cos(q[0])*sin(q[1])*sin(q[3]))) + fx*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + 
                fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - 
                fbet*cos(q[0]) + falp*sin(q[0]));

    dhdq(4,0) = K5*(fgam*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - fbet*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + 
                sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                d7*fx*sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                g_acc*lg7*m7*sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2]))) - 
                K4*(d7*fz*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*fx*sin(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                d7*fy*sin(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*lg7*m7*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])))*(fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) - fgam*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) - K6*(fgam*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                falp*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                fbet*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                d7*fz*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*cos(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*cos(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*lg7*m7*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2])))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - 
                K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + falp*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + K3*(g_acc*sin(q[1])*(lg7*m7*cos(q[2])*cos(q[4])*sin(q[5]) - 
                lg7*m7*cos(q[3])*sin(q[2])*sin(q[4])*sin(q[5])) + d7*fz*sin(q[5])*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])) + d7*fx*sin(q[5])*(cos(q[4])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[3])*sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                d5*sin(q[1])*sin(q[2])*sin(q[3])) - falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*sin(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + 
                d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*sin(q[1])*sin(q[2])) - 
                K1*(d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) 
                - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0])) - K2*(d7*fz*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[2]) - sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4])) + 
                d7*fx*cos(q[0])*sin(q[5])*(cos(q[4])*sin(q[1])*sin(q[2]) + cos(q[1])*sin(q[3])*sin(q[4]) + cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])) + d7*fy*sin(q[0])*sin(q[5])*(cos(q[4])*sin(q[1])*sin(q[2]) + cos(q[1])*sin(q[3])*sin(q[4]) 
                + cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])) - g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2])))*(fz*(d5*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + 
                d3*sin(q[1])) - fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) - fgam*cos(q[1]) - fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + 
                d3*sin(q[1])) - falp*cos(q[0])*sin(q[1]) - fbet*sin(q[0])*sin(q[1]) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1]));

    dhdq(5,0) = K5*(d7*fz*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*cos(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*cos(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*lg7*m7*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2])))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2]))) - K6*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))))*(falp*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fgam*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - fbet*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fx*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d7*fy*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                g_acc*lg7*m7*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + 
                K3*(g_acc*sin(q[1])*(lg7*m7*cos(q[2])*cos(q[5])*sin(q[4]) - lg7*m7*sin(q[2])*sin(q[3])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2])) + d7*fx*(cos(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) + d7*fz*(cos(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + 
                cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) + g_acc*sin(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + 
                lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + fgam*sin(q[1])*sin(q[2])) - K1*(d7*fy*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fx*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0])) - 
                K4*(d7*fz*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d7*fx*(sin(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) - 
                d7*fy*(sin(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))) + g_acc*lg7*m7*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))))*(fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) - fgam*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) + K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - falp*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                fbet*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))) + K2*(d7*fx*(cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) + cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d7*fy*(cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) + sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - d7*fz*(cos(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) - g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))))*(fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) - fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) - fgam*cos(q[1]) - fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + g_acc*m7*(d5*(cos(q[3])*sin(q[1]) 
                + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) - 
                falp*cos(q[0])*sin(q[1]) - fbet*sin(q[0])*sin(q[1]) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1]));

    dhdq(6,0) = 0;
    /*
    Can be computed using torque minimization - other gradient file 
    */
    return dhdq;
}

//***********************************Torque minimization function: CASE2 Gravity only ****************************************
//% dhdq(1,1) = simplify(transpose(diff(G,q1))*K*G);
MatrixXd TorqueOptCase2(double q[7])
{
    MatrixXd dhdq(7,1);		//Gradient of cost funtion
    
    dhdq(1,0) = K2*(g_acc*m5*(lg5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) + g_acc*m6*(lg6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) + 
                g_acc*m7*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - lg7*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])) + g_acc*lg3*m3*cos(q[1]) + g_acc*lg4*m4*cos(q[1]))*(g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - 
                lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1])) - K4*(g_acc*m7*(d5*(cos(q[1])*sin(q[3]) 
                + cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))*(g_acc*m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + g_acc*lg6*m6*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3]))) + K3*pow(g_acc,2)*cos(q[1])*sin(q[1])*pow((lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])),2) - K6*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - K5*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*pow(sin(q[5]),2)*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2]))*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2]));
        
    dhdq(2,0) = K3*pow(g_acc,2)*pow(sin(q[1]),2)*(d5*m7*cos(q[2])*sin(q[3]) + lg5*m5*cos(q[2])*sin(q[3]) + lg6*m6*cos(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*cos(q[5])*sin(q[3]) - lg7*m7*sin(q[2])*sin(q[4])*sin(q[5]) + 
                lg7*m7*cos(q[2])*cos(q[3])*cos(q[4])*sin(q[5]))*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + 
                lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) - K2*g_acc*cos(q[1])*(g_acc*m5*(d3*sin(q[1]) + lg5*cos(q[3])*sin(q[1]) + lg5*cos(q[1])*cos(q[2])*sin(q[3])) + g_acc*m6*(d3*sin(q[1]) + lg6*cos(q[3])*sin(q[1]) + 
                lg6*cos(q[1])*cos(q[2])*sin(q[3])) + g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - lg7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1]))*(lg5*m5*sin(q[2])*sin(q[3]) + 
                lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) - 
                K4*g_acc*sin(q[1])*sin(q[2])*(g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))*(d5*m7*cos(q[3]) + lg5*m5*cos(q[3]) + 
                lg6*m6*cos(q[3]) + lg7*m7*cos(q[3])*cos(q[5]) - lg7*m7*cos(q[4])*sin(q[3])*sin(q[5])) - K6*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*sin(q[1])*(cos(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))*(cos(q[2])*cos(q[5])*sin(q[4]) - sin(q[2])*sin(q[3])*sin(q[5]) + cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2])) + 
                K5*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*sin(q[1])*pow(sin(q[5]),2)*(cos(q[2])*cos(q[4]) - cos(q[3])*sin(q[2])*sin(q[4]))*(cos(q[4])*sin(q[1])*sin(q[2]) + cos(q[1])*sin(q[3])*sin(q[4]) + cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]));

    dhdq(3,0) = K4*(g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))*(g_acc*m7*(d5*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) + lg7*(cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))) + g_acc*lg5*m5*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) + g_acc*lg6*m6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - K2*(g_acc*m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + g_acc*lg6*m6*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])))*(g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) 
                + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1])) + K3*pow(g_acc,2)*pow(sin(q[1]),2)*(d5*m7*cos(q[3])*sin(q[2]) + lg5*m5*cos(q[3])*sin(q[2]) + lg6*m6*cos(q[3])*sin(q[2]) + 
                lg7*m7*cos(q[3])*cos(q[5])*sin(q[2]) - lg7*m7*cos(q[4])*sin(q[2])*sin(q[3])*sin(q[5]))*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) - K6*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + K5*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*sin(q[4])*pow(sin(q[5]),2)*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2]))*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]));

    dhdq(4,0) = K3*pow(g_acc,2)*pow(sin(q[1]),2)*(lg7*m7*cos(q[2])*cos(q[4])*sin(q[5]) - lg7*m7*cos(q[3])*sin(q[2])*sin(q[4])*sin(q[5]))*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) - K6*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2]))*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + K5*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*pow(sin(q[5]),2)*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2]))*(cos(q[4])*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + K2*g_acc*lg7*m7*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2]))*(g_acc*m7*(d5*(cos(q[3])*sin(q[1]) 
                + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + 
                g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1])) - K4*g_acc*lg7*m7*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))*(g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])));

    dhdq(5,0) = K3*pow(g_acc,2)*pow(sin(q[1]),2)*(lg7*m7*cos(q[2])*cos(q[5])*sin(q[4]) - lg7*m7*sin(q[2])*sin(q[3])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*cos(q[5])*sin(q[2]))*(lg5*m5*sin(q[2])*sin(q[3]) + 
                lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) - 
                K6*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])))*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - 
                K2*g_acc*lg7*m7*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])))*(g_acc*m7*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1]) - lg7*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + g_acc*m5*(lg5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*m6*(lg6*(cos(q[3])*sin(q[1]) 
                + cos(q[1])*cos(q[2])*sin(q[3])) + d3*sin(q[1])) + g_acc*lg3*m3*sin(q[1]) + g_acc*lg4*m4*sin(q[1])) - K4*g_acc*lg7*m7*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))*(g_acc*m7*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + lg7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + g_acc*lg5*m5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + g_acc*lg6*m6*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) + 
                K5*pow(g_acc,2)*pow(lg7,2)*pow(m7,2)*cos(q[5])*sin(q[5])*pow((sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])),2);

    dhdq(6,0) = 0;
    
    /*
    Can be computed using torque minimization - other gradient file 
    */
    
    return dhdq;
}

//***********************************Torque minimization function: CASE3 TauE only ****************************************
//dhdq(1,1) = simplify(transpose(diff(TauE,q1))*K*TauE);
MatrixXd TorqueOptCase3(double q[7])
{
    MatrixXd dhdq(7,1);		//Gradient of cost funtion
     
    dhdq(0,0) = K5*(falp*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                fbet*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                K4*(fy*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + fx*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + 
                d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + fbet*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                falp*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])))*(fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) 
                - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + 
                d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) - fz*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + K7*(fbet*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) 
                + falp*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))) - K3*(falp*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fbet*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])) - fx*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) 
                + cos(q[0])*cos(q[1])*sin(q[2]))))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - 
                falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*sin(q[1])*sin(q[2])) + 
                K1*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) 
                - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) 
                + d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0]))*(fy*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) 
                + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) 
                - cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - fx*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) 
                - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) 
                + cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + falp*cos(q[0]) + fbet*sin(q[0])) + K2*(fy*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) 
                - fx*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fbet*cos(q[0])*sin(q[1]) - falp*sin(q[0])*sin(q[1]))*(fx*(d3*cos(q[0])*cos(q[1]) - 
                d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fgam*cos(q[1]) - 
                fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + falp*cos(q[0])*sin(q[1]) + fbet*sin(q[0])*sin(q[1])) + K6*(fbet*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) 
                + falp*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fy*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                d7*fx*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))));

    dhdq(1,0) = K3*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - falp*(cos(q[2])*sin(q[0]) 
                + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                fgam*sin(q[1])*sin(q[2]))*(fx*(d7*(sin(q[5])*(cos(q[0])*cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[0])*cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[0])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                d5*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + fy*(d7*(sin(q[5])*(cos(q[2])*sin(q[0])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                d5*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + fz*(d7*(sin(q[5])*(cos(q[1])*cos(q[2])*sin(q[4]) + cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])) + cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])) + d5*cos(q[1])*sin(q[2])*sin(q[3])) + 
                fgam*cos(q[1])*sin(q[2]) + falp*cos(q[0])*sin(q[1])*sin(q[2]) + fbet*sin(q[0])*sin(q[1])*sin(q[2])) - K6*(falp*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - 
                sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + fbet*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - 
                cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + 
                fgam*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) - 
                d7*fx*(sin(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + cos(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - 
                cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) - d7*fy*(sin(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - 
                cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - K2*(fx*(d3*cos(q[0])*sin(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[0])*cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d5*cos(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + 
                fy*(d5*sin(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - 
                cos(q[5])*sin(q[0])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[0])*sin(q[1])) + fgam*sin(q[1]) + fz*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                d7*(sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])) - 
                falp*cos(q[0])*cos(q[1]) - fbet*cos(q[1])*sin(q[0]))*(fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) 
                - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d3*cos(q[1])*sin(q[0])) + fgam*cos(q[1]) - fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + falp*cos(q[0])*sin(q[1]) + fbet*sin(q[0])*sin(q[1])) - K4*(fgam*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3])) - fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + 
                fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) - fz*(d5*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - falp*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1])) + fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])))*(fgam*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) + 
                fx*(d7*(cos(q[5])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d5*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1]))) - fz*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + d7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) - falp*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + fy*(d7*(cos(q[5])*(cos(q[1])*sin(q[0])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3]))) + d5*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1]))) - 
                fbet*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3]))) + K5*(falp*(sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[0])*cos(q[4])*sin(q[1])*sin(q[2])) 
                + fbet*(sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) + cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])) - fgam*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - 
                cos(q[1])*cos(q[4])*sin(q[2])) - d7*fz*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2])) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + 
                cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[0])*cos(q[4])*sin(q[1])*sin(q[2])) + d7*fy*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) + 
                cos(q[4])*sin(q[0])*sin(q[1])*sin(q[2])))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(falp*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + 
                cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + fbet*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - 
                sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + fgam*(sin(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + K1*(fx*(d7*(cos(q[5])*(cos(q[1])*cos(q[3])*sin(q[0]) - 
                cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])) - sin(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[1])*cos(q[3])*sin(q[0]) - 
                cos(q[2])*sin(q[0])*sin(q[1])*sin(q[3])) + d3*cos(q[1])*sin(q[0])) - fy*(d7*(cos(q[5])*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) - sin(q[5])*(cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[3]) + 
                cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[0])*sin(q[1])*sin(q[2])*sin(q[4]))) + d5*(cos(q[0])*cos(q[1])*cos(q[3]) - cos(q[0])*cos(q[2])*sin(q[1])*sin(q[3])) + 
                d3*cos(q[0])*cos(q[1])))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) 
                - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) 
                + d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0]));

    dhdq(2,0) = K4*(fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3]))) + fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) - fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - falp*(sin(q[3])*(sin(q[0])*sin(q[2]) 
                - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])))*(fy*(d7*(cos(q[3])*cos(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) - cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                d5*cos(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - fx*(d7*(cos(q[3])*cos(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) - cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) + d5*cos(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + fz*(d7*(cos(q[3])*cos(q[5])*sin(q[1])*sin(q[2]) - cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])) + 
                d5*cos(q[3])*sin(q[1])*sin(q[2])) - falp*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fgam*sin(q[1])*sin(q[2])*sin(q[3])) - 
                K1*(fx*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fy*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0])) + K3*(falp*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - fbet*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + fx*(d7*(sin(q[5])*(sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + cos(q[3])*cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) + 
                cos(q[5])*sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) + d5*sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])) + cos(q[3])*cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0]))) + cos(q[5])*sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0]))) + d5*sin(q[3])*(cos(q[0])*sin(q[2]) 
                + cos(q[1])*cos(q[2])*sin(q[0]))) - fz*(d7*(sin(q[5])*(sin(q[1])*sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[2])*cos(q[5])*sin(q[1])*sin(q[3])) - d5*cos(q[2])*sin(q[1])*sin(q[3])) + 
                fgam*cos(q[2])*sin(q[1]))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - 
                falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*sin(q[1])*sin(q[2])) - 
                K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(falp*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + 
                cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3]))) - K6*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) 
                - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))))*(falp*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + 
                cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d7*fx*(cos(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + d7*fz*(cos(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) - 
                sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5]))) + K5*(falp*(cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[3])*sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                fbet*(cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*(cos(q[2])*cos(q[4])*sin(q[1]) - 
                cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])) + d7*fz*sin(q[5])*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])) + d7*fx*sin(q[5])*(cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[3])*sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                K2*(fx*(d7*(cos(q[0])*sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[0])*cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*cos(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                fy*(d7*(sin(q[0])*sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[0])*sin(q[1])*sin(q[2])*sin(q[3])) + 
                fz*(d7*(sin(q[5])*(cos(q[1])*cos(q[2])*sin(q[4]) + cos(q[1])*cos(q[3])*cos(q[4])*sin(q[2])) + cos(q[1])*cos(q[5])*sin(q[2])*sin(q[3])) + d5*cos(q[1])*sin(q[2])*sin(q[3])))*(fx*(d3*cos(q[0])*cos(q[1]) - 
                d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fgam*cos(q[1]) - 
                fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + falp*cos(q[0])*sin(q[1]) + fbet*sin(q[0])*sin(q[1]));

    dhdq(3,0) = K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1]))) - fbet*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])))) - K4*(fgam*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - fx*(d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) 
                - cos(q[0])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + fy*(d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                cos(q[4])*sin(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) + d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))) + fz*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d7*(cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])))) + falp*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - fbet*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])))*(fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3]))) + fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) - fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - falp*(sin(q[3])*(sin(q[0])*sin(q[2]) 
                - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) - 
                K2*(fy*(d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[3])*cos(q[4]) - cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])) + cos(q[5])*sin(q[0])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))) + d5*sin(q[0])*(cos(q[1])*sin(q[3]) 
                + cos(q[2])*cos(q[3])*sin(q[1]))) - fz*(d7*(sin(q[5])*(cos(q[3])*cos(q[4])*sin(q[1]) + cos(q[1])*cos(q[2])*cos(q[4])*sin(q[3])) + cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]))) + 
                d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]))) + fx*(d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[3])*cos(q[4]) - cos(q[2])*cos(q[4])*sin(q[1])*sin(q[3])) + cos(q[0])*cos(q[5])*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1]))) + d5*cos(q[0])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1]))))*(fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) 
                + fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fgam*cos(q[1]) - fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - 
                d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + 
                falp*cos(q[0])*sin(q[1]) + fbet*sin(q[0])*sin(q[1])) + K6*(fgam*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1]))) - fbet*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) - d7*fz*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - 
                d7*fx*(sin(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1]))) + d7*fy*(sin(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) 
                - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + 
                K5*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*sin(q[4])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - falp*sin(q[4])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) 
                + fbet*sin(q[4])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*fz*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                d7*fx*sin(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*fy*sin(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))) + K1*(fy*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3]))) + 
                fx*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - 
                fbet*cos(q[0]) + falp*sin(q[0])) + K3*(fy*(d7*(cos(q[3])*cos(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) - cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                d5*cos(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - fx*(d7*(cos(q[3])*cos(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) - cos(q[4])*sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) + d5*cos(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + fz*(d7*(cos(q[3])*cos(q[5])*sin(q[1])*sin(q[2]) - cos(q[4])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])) + 
                d5*cos(q[3])*sin(q[1])*sin(q[2])))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - 
                falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*sin(q[1])*sin(q[2]));

    dhdq(4,0) = K5*(fgam*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - fbet*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + 
                sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                d7*fx*sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                K2*(d7*fz*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[2]) - sin(q[1])*sin(q[3])*sin(q[4]) + cos(q[1])*cos(q[2])*cos(q[3])*sin(q[4])) + d7*fx*cos(q[0])*sin(q[5])*(cos(q[4])*sin(q[1])*sin(q[2]) + cos(q[1])*sin(q[3])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])) + d7*fy*sin(q[0])*sin(q[5])*(cos(q[4])*sin(q[1])*sin(q[2]) + cos(q[1])*sin(q[3])*sin(q[4]) + cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])))*(fx*(d3*cos(q[0])*cos(q[1]) - 
                d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - 
                sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fgam*cos(q[1]) - 
                fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + falp*cos(q[0])*sin(q[1]) + fbet*sin(q[0])*sin(q[1])) + K4*(d7*fz*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                d7*fx*sin(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*fy*sin(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])))*(fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3]))) + fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) - fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - falp*(sin(q[3])*(sin(q[0])*sin(q[2]) 
                - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) - 
                K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + falp*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - K1*(d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - 
                fbet*cos(q[0]) + falp*sin(q[0])) + K3*(d7*fz*sin(q[5])*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4])) + d7*fx*sin(q[5])*(cos(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[3])*sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(cos(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - 
                falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*sin(q[1])*sin(q[2])) - 
                K6*(fgam*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + falp*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fbet*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + d7*fz*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + 
                d7*fx*cos(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                d7*fy*cos(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + 
                d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))));

    dhdq(5,0) = K5*(d7*fz*cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*cos(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*cos(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))*(fgam*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
                cos(q[4])*sin(q[1])*sin(q[2])) - fbet*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                falp*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + 
                d7*fz*sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[1])*sin(q[2])) + d7*fx*sin(q[5])*(sin(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*sin(q[5])*(sin(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + K6*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) 
                - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - d7*fx*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                d7*fy*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) + d7*fz*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))))*(fgam*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) - falp*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + fbet*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) 
                - d7*fx*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fy*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))) - 
                d7*fz*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) + 
                K4*(d7*fz*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d7*fx*(sin(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) - 
                d7*fy*(sin(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + cos(q[4])*cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + 
                cos(q[3])*sin(q[0])*sin(q[1]))))*(fgam*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - fx*(d7*(cos(q[5])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) - 
                cos(q[4])*sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1]))) + d5*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3]))) + fy*(d7*(cos(q[5])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) + d5*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3]))) - fz*(d5*(cos(q[1])*sin(q[3]) + 
                cos(q[2])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + cos(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])))) - falp*(sin(q[3])*(sin(q[0])*sin(q[2]) 
                - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + fbet*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1]))) - 
                K1*(d7*fy*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + 
                cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + d7*fx*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))))*(fy*(d5*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - 
                cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + 
                cos(q[0])*cos(q[1])*sin(q[2])))) - d3*cos(q[0])*sin(q[1])) + fx*(d5*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + d7*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])))) + d3*sin(q[0])*sin(q[1])) - fbet*cos(q[0]) + falp*sin(q[0])) + K7*(fgam*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) 
                - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + falp*(cos(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) + 
                sin(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) - 
                fbet*(cos(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) + sin(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2])))))*(fgam*(cos(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) + 
                sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - falp*(sin(q[5])*(sin(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[0])*cos(q[3])*sin(q[1])) - 
                cos(q[5])*(cos(q[4])*(cos(q[3])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])))) + 
                fbet*(sin(q[5])*(sin(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[0])*sin(q[1])) - cos(q[5])*(cos(q[4])*(cos(q[3])*(cos(q[0])*sin(q[2]) + cos(q[1])*cos(q[2])*sin(q[0])) - 
                sin(q[0])*sin(q[1])*sin(q[3])) + sin(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))))) + K3*(d7*fx*(cos(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - 
                cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d7*fy*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) + 
                d7*fz*(cos(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[5])))*(fz*(d7*(sin(q[5])*(cos(q[2])*sin(q[1])*sin(q[4]) + 
                cos(q[3])*cos(q[4])*sin(q[1])*sin(q[2])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[3])) + d5*sin(q[1])*sin(q[2])*sin(q[3])) - falp*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2])) + fbet*(cos(q[0])*cos(q[2]) - 
                cos(q[1])*sin(q[0])*sin(q[2])) + fx*(d7*(sin(q[5])*(sin(q[4])*(sin(q[0])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[3])*cos(q[4])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - 
                cos(q[5])*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - d5*sin(q[3])*(cos(q[2])*sin(q[0]) + cos(q[0])*cos(q[1])*sin(q[2]))) - fy*(d7*(sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[2]) + 
                cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[3])*cos(q[4])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[0])*cos(q[2]) - cos(q[1])*sin(q[0])*sin(q[2]))) - d5*sin(q[3])*(cos(q[0])*cos(q[2]) 
                - cos(q[1])*sin(q[0])*sin(q[2]))) + fgam*sin(q[1])*sin(q[2])) - K2*(d7*fx*(cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) + 
                cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d7*fy*(cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) + 
                sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) - d7*fz*(cos(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) + 
                sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))))*(fx*(d3*cos(q[0])*cos(q[1]) - d7*(cos(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + 
                cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - cos(q[0])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d5*cos(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + 
                fy*(d5*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - d7*(sin(q[0])*sin(q[5])*(cos(q[1])*cos(q[4])*sin(q[3]) - sin(q[1])*sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])*sin(q[1])) - 
                cos(q[5])*sin(q[0])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])*sin(q[0])) + fgam*cos(q[1]) - fz*(d5*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])) - 
                d7*(sin(q[5])*(cos(q[1])*sin(q[2])*sin(q[4]) + cos(q[4])*sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])) - cos(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3]))) + d3*sin(q[1])) + 
                falp*cos(q[0])*sin(q[1]) + fbet*sin(q[0])*sin(q[1]));
                
    dhdq(6,0) = 0;
    
    //NOTE: both equations are same
    /*
    Can be computed using torque minimization - other gradient file 
    */

    return dhdq;
}

//***********************************Torque minimization function: CASE4 PROPOSED ****************************************
//dhdq(1,1) = simplify(transpose(diff(TauE,q1))*K*TauE);
MatrixXd TorqueOptCase4(double q[7])
{
    MatrixXd dhdq(7,1);		//Gradient of cost funtion
     
dhdq(0,0) = 0;
dhdq(1,0) = K6*pow((LL6*m6 + L6*(m6 + m7)),2)*(sin(q[1])*(cos(q[3])*sin(q[5]) + cos(q[4])*cos(q[5])*sin(q[3])) + 
			cos(q[1])*cos(q[2])*(sin(q[3])*sin(q[5]) - cos(q[3])*cos(q[4])*cos(q[5])) + 
			cos(q[1])*cos(q[5])*sin(q[2])*sin(q[4]))*(cos(q[2])*sin(q[1])*(sin(q[3])*sin(q[5]) - cos(q[3])*cos(q[4])*cos(q[5])) - 
			cos(q[1])*(cos(q[3])*sin(q[5]) + cos(q[4])*cos(q[5])*sin(q[3])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])) - 
			K2*(cos(q[1])*(cos(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + L2*m2 + L2*m3 + L2*m4 + L2*m5 + 
			L2*m6 + L2*m7 + LL2*m2 - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))))*(sin(q[1])*(cos(q[2])*(sin(q[3])*(LL4*m4 + 
			cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - cos(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + 
			L4*(m4 + m5 + m6 + m7)) + L2*m2 + L2*m3 + L2*m4 + L2*m5 + L2*m6 + L2*m7 + LL2*m2 - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + 
			L6*(m6 + m7)))) - K4*(cos(q[1])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + 
			m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))))*(sin(q[1])*(sin(q[3])*(LL4*m4 + 
			cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			cos(q[1])*cos(q[2])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) - 
			cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)))) + K3*cos(q[1])*sin(q[1])*pow((sin(q[2])*(sin(q[3])*(LL4*m4 + 
			cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			cos(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))),2) - K5*pow(sin(q[5]),2)*(sin(q[4])*(sin(q[1])*sin(q[3]) - 
			cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2]))*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) 
			+ cos(q[4])*sin(q[1])*sin(q[2]))*pow((LL6*m6 + L6*(m6 + m7)),2);
dhdq(2,0) = K3*pow(sin(q[1]),2)*(sin(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + 
			m7)))*(cos(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			K2*cos(q[1])*(cos(q[1])*(cos(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + L2*m2 + L2*m3 + L2*m4 + L2*m5 + 
			L2*m6 + L2*m7 + LL2*m2 - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))))*(sin(q[2])*(sin(q[3])*(LL4*m4 + 
			cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			cos(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - K6*pow((LL6*m6 + L6*(m6 + m7)),2)*(sin(q[1])*sin(q[2])*(sin(q[3])*sin(q[5]) 
			- cos(q[3])*cos(q[4])*cos(q[5])) - cos(q[2])*cos(q[5])*sin(q[1])*sin(q[4]))*(cos(q[2])*sin(q[1])*(sin(q[3])*sin(q[5]) - 
			cos(q[3])*cos(q[4])*cos(q[5])) - cos(q[1])*(cos(q[3])*sin(q[5]) + cos(q[4])*cos(q[5])*sin(q[3])) + 
			cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])) + K5*pow(sin(q[5]),2)*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
			cos(q[4])*sin(q[1])*sin(q[2]))*(cos(q[2])*cos(q[4])*sin(q[1]) - cos(q[3])*sin(q[1])*sin(q[2])*sin(q[4]))*pow((LL6*m6 + L6*(m6 + 
			m7)),2) - K4*sin(q[1])*sin(q[2])*(cos(q[1])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + 
			m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))))*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 
			+ L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)));
dhdq(3,0) = K4*(cos(q[1])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + 
			m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))))*(cos(q[1])*(cos(q[3])*(LL4*m4 + 
			cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			cos(q[2])*sin(q[1])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)))) - K2*(cos(q[1])*(cos(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + 
			L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + 
			L4*(m4 + m5 + m6 + m7)) + L2*m2 + L2*m3 + L2*m4 + L2*m5 + L2*m6 + L2*m7 + LL2*m2 - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + 
			L6*(m6 + m7))))*(sin(q[1])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - cos(q[1])*cos(q[2])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + 
			m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)))) + K6*pow((LL6*m6 + L6*(m6 + 
			m7)),2)*(cos(q[1])*(sin(q[3])*sin(q[5]) - cos(q[3])*cos(q[4])*cos(q[5])) + cos(q[2])*sin(q[1])*(cos(q[3])*sin(q[5]) + 
			cos(q[4])*cos(q[5])*sin(q[3])))*(cos(q[2])*sin(q[1])*(sin(q[3])*sin(q[5]) - cos(q[3])*cos(q[4])*cos(q[5])) - 
			cos(q[1])*(cos(q[3])*sin(q[5]) + cos(q[4])*cos(q[5])*sin(q[3])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4])) + 
			K3*pow(sin(q[1]),2)*sin(q[2])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) - 
			cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)))*(sin(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + 
			L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + 
			L6*(m6 + m7))) + K5*sin(q[4])*pow(sin(q[5]),2)*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
			cos(q[4])*sin(q[1])*sin(q[2]))*pow((LL6*m6 + L6*(m6 + m7)),2)*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]));
dhdq(4,0) = K3*pow(sin(q[1]),2)*(sin(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + 
			m7)))*(cos(q[2])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) - cos(q[3])*sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) 
			- K2*(cos(q[1])*(cos(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + L2*m2 + L2*m3 + L2*m4 + L2*m5 + 
			L2*m6 + L2*m7 + LL2*m2 - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + 
			m7))))*(cos(q[1])*(cos(q[4])*sin(q[2])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) + cos(q[2])*cos(q[3])*sin(q[4])*sin(q[5])*(LL6*m6 + 
			L6*(m6 + m7))) - sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - K4*(cos(q[1])*(sin(q[3])*(LL4*m4 + 
			cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			cos(q[2])*sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) - 
			cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))))*(cos(q[1])*cos(q[3])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) - 
			cos(q[2])*sin(q[1])*sin(q[3])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + K6*pow((LL6*m6 + L6*(m6 + 
			m7)),2)*(cos(q[2])*sin(q[1])*(sin(q[3])*sin(q[5]) - cos(q[3])*cos(q[4])*cos(q[5])) - cos(q[1])*(cos(q[3])*sin(q[5]) + 
			cos(q[4])*cos(q[5])*sin(q[3])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]))*(cos(q[4])*cos(q[5])*sin(q[1])*sin(q[2]) + 
			cos(q[1])*cos(q[5])*sin(q[3])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[5])*sin(q[1])*sin(q[4])) + 
			K5*pow(sin(q[5]),2)*(sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
			cos(q[4])*sin(q[1])*sin(q[2]))*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
			sin(q[1])*sin(q[2])*sin(q[4]))*pow((LL6*m6 + L6*(m6 + m7)),2);
dhdq(5,0) = K5*cos(q[5])*sin(q[5])*pow((sin(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) + 
			cos(q[4])*sin(q[1])*sin(q[2])),2)*pow((LL6*m6 + L6*(m6 + m7)),2) - K2*(cos(q[1])*(cos(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + 
			L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			sin(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + 
			L4*(m4 + m5 + m6 + m7)) + L2*m2 + L2*m3 + L2*m4 + L2*m5 + L2*m6 + L2*m7 + LL2*m2 - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + 
			L6*(m6 + m7))))*(cos(q[1])*(cos(q[2])*(sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) - cos(q[3])*cos(q[4])*cos(q[5])*(LL6*m6 + 
			L6*(m6 + m7))) + cos(q[5])*sin(q[2])*sin(q[4])*(LL6*m6 + L6*(m6 + m7))) + sin(q[1])*(cos(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) 
			+ cos(q[4])*cos(q[5])*sin(q[3])*(LL6*m6 + L6*(m6 + m7)))) - K6*pow((LL6*m6 + L6*(m6 + 
			m7)),2)*(cos(q[2])*sin(q[1])*(sin(q[3])*sin(q[5]) - cos(q[3])*cos(q[4])*cos(q[5])) - cos(q[1])*(cos(q[3])*sin(q[5]) + 
			cos(q[4])*cos(q[5])*sin(q[3])) + cos(q[5])*sin(q[1])*sin(q[2])*sin(q[4]))*(cos(q[1])*(cos(q[3])*cos(q[5]) - 
			cos(q[4])*sin(q[3])*sin(q[5])) - cos(q[2])*sin(q[1])*(cos(q[5])*sin(q[3]) + cos(q[3])*cos(q[4])*sin(q[5])) + 
			sin(q[1])*sin(q[2])*sin(q[4])*sin(q[5])) - K3*pow(sin(q[1]),2)*(sin(q[2])*(sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) - 
			cos(q[3])*cos(q[4])*cos(q[5])*(LL6*m6 + L6*(m6 + m7))) - cos(q[2])*cos(q[5])*sin(q[4])*(LL6*m6 + L6*(m6 + 
			m7)))*(sin(q[2])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) - 
			K4*(cos(q[1])*(sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) - cos(q[3])*cos(q[4])*cos(q[5])*(LL6*m6 + L6*(m6 + m7))) + 
			cos(q[2])*sin(q[1])*(cos(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7)) + cos(q[4])*cos(q[5])*sin(q[3])*(LL6*m6 + L6*(m6 + 
			m7))))*(cos(q[1])*(sin(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + m7)) + L4*(m4 + m5 + m6 + m7)) + 
			cos(q[3])*cos(q[4])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))) + cos(q[2])*sin(q[1])*(cos(q[3])*(LL4*m4 + cos(q[5])*(LL6*m6 + L6*(m6 + 
			m7)) + L4*(m4 + m5 + m6 + m7)) - cos(q[4])*sin(q[3])*sin(q[5])*(LL6*m6 + L6*(m6 + m7))));
dhdq(6,0) = 0;
    
    //NOTE: both equations are same
    /*
    Can be computed using torque minimization - other gradient file 
    */

    return dhdq;
}

//***********************************Torque minimization function: CASE5 G2 ****************************************
//dhdq(1,1) = simplify(transpose(diff(TauE,q1))*K*TauE);
MatrixXd TorqueOptCase5(double q[7])
{
    MatrixXd dhdq(7,1);		//Gradient of cost funtion
    /* 
    dhdq(0,0) = 0;
    dhdq(1,0) = - 1*m5*(lg5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) - 1*m6*(lg6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + 
                d3*cos(q[1])) - 1*m7*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - lg7*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])) - 1*lg3*m3*cos(q[1]) - 1*lg4*m4*cos(q[1]);
    dhdq(2,0) = 1*cos(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5]));
    dhdq(3,0) = 1*m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + 1*lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                1*lg6*m6*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]));
    dhdq(4,0) = -1*lg7*m7*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2]));
    dhdq(5,0) = 1*lg7*m7*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])));
    dhdq(6,0) = 0;
    */ 
    dhdq(0,0) = 0;
    dhdq(1,0) = -(- 1*m5*(lg5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) - 1*m6*(lg6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + 
                d3*cos(q[1])) - 1*m7*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - lg7*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - 
                sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])) - 1*lg3*m3*cos(q[1]) - 1*lg4*m4*cos(q[1]));
    dhdq(2,0) = -(1*cos(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])));
    dhdq(3,0) = -(1*m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + 1*lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                1*lg6*m6*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])));
    dhdq(4,0) = -(-1*lg7*m7*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2])));
    dhdq(5,0) = -(1*lg7*m7*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3]))));
    dhdq(6,0) = 0;

return dhdq;
}

//***********************************Torque minimization function: CASE6 G2 G4 ****************************************
//dhdq(1,1) = simplify(transpose(diff(TauE,q1))*K*TauE);
MatrixXd TorqueOptCase6(double q[7])
{
    MatrixXd dhdq(7,1);		//Gradient of cost funtion

    dhdq(0,0) = 0;
    dhdq(1,0) = eta2*(m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                lg6*m6*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]))) - eta1*(m5*(lg5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) + 
                m6*(lg6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + d3*cos(q[1])) + m7*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - 
                lg7*(sin(q[5])*(cos(q[4])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - sin(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(cos(q[1])*cos(q[3]) - 
                cos(q[2])*sin(q[1])*sin(q[3]))) + d3*cos(q[1])) + lg3*m3*cos(q[1]) + lg4*m4*cos(q[1]));
    dhdq(2,0) = eta1*cos(q[1])*(lg5*m5*sin(q[2])*sin(q[3]) + lg6*m6*sin(q[2])*sin(q[3]) + d5*m7*sin(q[2])*sin(q[3]) + lg7*m7*cos(q[5])*sin(q[2])*sin(q[3]) + 
                lg7*m7*cos(q[2])*sin(q[4])*sin(q[5]) + lg7*m7*cos(q[3])*cos(q[4])*sin(q[2])*sin(q[5])) + eta2*sin(q[1])*sin(q[2])*(d5*m7*cos(q[3]) + lg5*m5*cos(q[3]) + 
                lg6*m6*cos(q[3]) + lg7*m7*cos(q[3])*cos(q[5]) - lg7*m7*cos(q[4])*sin(q[3])*sin(q[5]));
    dhdq(3,0) = eta1*(m7*(d5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + lg7*(cos(q[5])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                cos(q[4])*sin(q[5])*(cos(q[3])*sin(q[1]) + cos(q[1])*cos(q[2])*sin(q[3])))) + lg5*m5*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + 
                lg6*m6*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3]))) - eta2*(m7*(d5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + 
                lg7*(cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - cos(q[4])*sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])))) + 
                lg5*m5*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) + lg6*m6*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])));
    dhdq(4,0) = eta2*lg7*m7*sin(q[4])*sin(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3])) - eta1*lg7*m7*sin(q[5])*(sin(q[4])*(sin(q[1])*sin(q[3]) - 
                cos(q[1])*cos(q[2])*cos(q[3])) - cos(q[1])*cos(q[4])*sin(q[2]));
    dhdq(5,0) = eta2*lg7*m7*(sin(q[5])*(cos(q[1])*sin(q[3]) + cos(q[2])*cos(q[3])*sin(q[1])) - cos(q[4])*cos(q[5])*(cos(q[1])*cos(q[3]) - cos(q[2])*sin(q[1])*sin(q[3]))) 
                + eta1*lg7*m7*(cos(q[5])*(cos(q[4])*(sin(q[1])*sin(q[3]) - cos(q[1])*cos(q[2])*cos(q[3])) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(cos(q[3])*sin(q[1]) + 
                cos(q[1])*cos(q[2])*sin(q[3])));
    dhdq(6,0) = 0;

    return dhdq;
}



void inversedynamics_torqueopt()
{
    //Input desired cartesian pos, vel, acc: pd, dpd, ddpd

    if(savedata<10)
        {
            DataSave(runcount);
        }
    //Update current joint angle
    current_q(0,0) = current_joint_states[0];
    current_q(1,0) = current_joint_states[1];
    current_q(2,0) = current_joint_states[2];
    current_q(3,0) = current_joint_states[3];
    current_q(4,0) = current_joint_states[4];
    current_q(5,0) = current_joint_states[5];
    current_q(6,0) = current_joint_states[6];
    //Update current joint angular velocity
    current_dq = (current_q - prev_q)/0.001;
    current_ddq = (current_dq - prev_dq)/0.001;

    current_joint_vel[0] = current_dq(0,0);
    current_joint_vel[1] = current_dq(1,0);
    current_joint_vel[2] = current_dq(2,0);
    current_joint_vel[3] = current_dq(3,0);
    current_joint_vel[4] = current_dq(4,0);
    current_joint_vel[5] = current_dq(5,0);
    current_joint_vel[6] = current_dq(6,0);


    //current cartesian position
    jointpos=forwd7(current_joint_states);
    currentp(0,0) = jointpos(0,0);
    currentp(1,0) = jointpos(1,0);
    currentp(2,0) = jointpos(2,0);
    currentp(3,0) = jointpos(3,0);
    currentp(4,0) = jointpos(4,0);
    currentp(5,0) = jointpos(5,0);
    
    //current cartesian velocity
    //Jacob = Jacobian_Matrix(current_joint_states);
    //currentdp = Jacob*current_dq;
    currentdp = (currentp - prevp)/0.001;

    //Compute cartersian errors
    pderror = pd - currentp;        //pos error
    dpderror = dpd - currentdp;     //vel error

    //Max error recording
    if(fabs(pderror(0,0))>fabs(maxperror(0,0))) maxperror(0,0) = pderror(0,0);  //x
    if(fabs(pderror(1,0))>fabs(maxperror(1,0))) maxperror(1,0) = pderror(1,0);  //y
    if(fabs(pderror(2,0))>fabs(maxperror(2,0))) maxperror(2,0) = pderror(2,0);  //z


    //Desired joint angle:
    pdesired[0] = pd(0,0);
    pdesired[1] = pd(1,0);
    pdesired[2] = pd(2,0);
    pdesired[3] = pd(3,0);
    pdesired[4] = pd(4,0);
    pdesired[5] = pd(5,0);
    InvK7_1(pdesired, SI_FIX);
    qd(0,0) = q[0];
    qd(1,0) = q[1];
    qd(2,0) = q[2];
    qd(3,0) = q[3];
    qd(4,0) = q[4];
    qd(5,0) = q[5];
    qd(6,0) = q[6];
    //Desired joint velocity:
    dqdesired[0] = dqd(0,0);
    dqdesired[1] = dqd(1,0);
    dqdesired[2] = dqd(2,0);
    dqdesired[3] = dqd(3,0);
    dqdesired[4] = dqd(4,0);
    dqdesired[5] = dqd(5,0);
    dqdesired[6] = dqd(6,0);

    //Update the differentiate of Jacobian
    Jacob = Jacobian_Matrix(q);
    DiffJacob = DiffJacobian_Matrix(q, dqdesired);
    Coriolis = Coriolis_Matrix(q, dqdesired);
    Mass = Mass_Matrix(q);
    Gravity = Gravity_Matrix(q);
    //Update pseudo-inverse of Jacobian:
    pinv_Jacob = Jacob.transpose()*(Jacob*Jacob.transpose()).inverse();
    //Mpinv_Jacob = Mass.inverse()*Jacob.transpose()*(Jacob*Mass.inverse()*Jacob.transpose()).inverse();
    
    lamda0 = 0.008;
    singular_wt = 0.1;

    Manipulability = Jacob*Jacob.transpose();
    //cout<<Manipulability<<endl;
    singular_w = sqrt(Manipulability.determinant());

    if (singular_w>=singular_wt)
    {
        lamda = lamda0;
    }
    else
    {
        lamda = lamda0*(1-(singular_w/singular_wt));
    }
    
    //printf("lamda= %.f, singular_w= %.f \n",lamda, singular_w);
    Mpinv_Jacob = Mass.inverse()*Jacob.transpose()*(Jacob*Mass.inverse()*Jacob.transpose() + lamda*I6).inverse();   //singularity avoindance
    Diff_Mpinv_Jacob = (Mpinv_Jacob - Mpinv_Jacob_Prev)/freq;


    alpha1 = -0.000082;  //-0.000087
    alpha2 = 0.7;

    dy1 = TorqueOptCase4(q);   //NEW PROPOSED
    dy2 = dy2_prev + freq*ddy2_prev;
    dy = alpha1*dy1 + alpha2*dy2;
    
    //Desired joint vel
    //dqd = Mpinv_Jacob*dpd;
    dqd = Mpinv_Jacob*dpd + (I7-pinv_Jacob*Jacob)*dy;

    //Desired joint acc
    //* 
    ddpd(0,0) = 0;
    ddpd(1,0) = 0;
    ddpd(2,0) = 0;
    ddpd(3,0) = 0;
    ddpd(4,0) = 0;
    ddpd(5,0) = 0;
    //*/

    ddy1 = (dy1 + dy1_prev)/freq;
    ddy2 = - Mass.inverse()*(Coriolis);
    ddy = alpha1*ddy1 + alpha2*ddy2;

    //Null space velocity error
    e_Ndot = (I7 - pinv_Jacob*Jacob)*(dqd - current_dq);//   ddy - alpha1*ddy1_prev - alpha2*ddy2_prev;
    //Null space acceleration
    phi_N = (I7 - Mpinv_Jacob*Jacob)*(ddy + 0.0152*e_Ndot);// - (Mpinv_Jacob*DiffJacob*Mpinv_Jacob + Diff_Mpinv_Jacob)*Jacob*(dqd - current_dq);

    //ddqd = Mpinv_Jacob*(ddpd + kp_matrix_6*(pd - currentp) + kd_matrix_6*(dpd - currentdp) - DiffJacob*dqd);
    ddqd = Mpinv_Jacob*(ddpd + kp_matrix_6*(pd - currentp) + kd_matrix_6*(dpd - currentdp) - DiffJacob*dqd) + phi_N;

    //Compute torque and apply the controller
    Tau_I = Mass*ddqd + Coriolis + Gravity;

    //Backup for next step:
    dy1_prev = dy1;
    dy2_prev = dy2;
    ddy2_prev = ddy2;
    ddy1_prev = ddy1;
    Mpinv_Jacob_Prev = Mpinv_Jacob;
    pinv_Jacob_Prev = pinv_Jacob;
    prevp = currentp;
    ddqd_prev = ddqd;
    dqd_prev = dqd;
    dpd_prev = dpd;

    //prev_dq = qd;
    prev_q(0,0) = current_joint_states[0];
    prev_q(1,0) = current_joint_states[1];
    prev_q(2,0) = current_joint_states[2];
    prev_q(3,0) = current_joint_states[3];
    prev_q(4,0) = current_joint_states[4];
    prev_q(5,0) = current_joint_states[5];
    prev_q(6,0) = current_joint_states[6];

    //Torque filter for torque limit avoidance
    for (int i=0; i<7; i++)
    {
        if(Tau_I(i,0) >= T_limit[i]) Tau_I(i,0) = T_limit[i];
        else if (Tau_I(i,0) <= -T_limit[i]) Tau_I(i,0) = -T_limit[i];
    }
    command_tau[0] = Tau_I(0,0);
    command_tau[1] = Tau_I(1,0);
    command_tau[2] = Tau_I(2,0);
    command_tau[3] = Tau_I(3,0);
    command_tau[4] = Tau_I(4,0);
    command_tau[5] = Tau_I(5,0);
    command_tau[6] = Tau_I(6,0);

    //Compute norm torque, efficiency, velocity & acc norm;
    drtorque = drtorque + Tau_I.transpose()*Tau_I*freq;
    taunorm = sqrt(pow(Tau_I(0,0),2) + pow(Tau_I(1,0),2) + pow(Tau_I(2,0),2) + pow(Tau_I(3,0),2) + pow(Tau_I(4,0),2) + pow(Tau_I(5,0),2) + pow(Tau_I(6,0),2));
    sumtaunorm = sumtaunorm + taunorm;
    velocitynorm = current_dq.norm();
    accelerationnorm = ddqd.norm();
    
    //Send torque command
    jointt_publish(command_tau);

    
    
    //Compute current Cartesian state
    jointpos=forwd7(current_joint_states);
    printf("x= %.4f y= %.4f z= %.4f alpha= %.4f beta= %.4f gamma= %.4f \n",jointpos(0,0),jointpos(1,0),jointpos(2,0),jointpos(3,0),jointpos(4,0),jointpos(5,0));

}

void inversedynamics_torqueopt1()
{
    //Input desired cartesian pos, vel, acc: pd, dpd, ddpd

    if(savedata<10)
        {
            DataSave(runcount);
            
        }
    //Update current joint angle
    current_q(0,0) = current_joint_states[0];
    current_q(1,0) = current_joint_states[1];
    current_q(2,0) = current_joint_states[2];
    current_q(3,0) = current_joint_states[3];
    current_q(4,0) = current_joint_states[4];
    current_q(5,0) = current_joint_states[5];
    current_q(6,0) = current_joint_states[6];
    //Update current joint angular velocity
    current_dq = (current_q - prev_q)/freq;
    current_ddq = (current_dq - prev_dq)/freq;
    prev_q = current_q;     //record current joint angle for the next step
    prev_dq = current_dq;   //record current joint vel angle for the next step

    current_joint_vel[0] = current_dq(0,0);
    current_joint_vel[1] = current_dq(1,0);
    current_joint_vel[2] = current_dq(2,0);
    current_joint_vel[3] = current_dq(3,0);
    current_joint_vel[4] = current_dq(4,0);
    current_joint_vel[5] = current_dq(5,0);
    current_joint_vel[6] = current_dq(6,0);


    //current cartesian position
    jointpos=forwd7(current_joint_states);
    //Compute current Cartesian state
    printf("x= %.4f y= %.4f z= %.4f alpha= %.4f beta= %.4f gamma= %.4f \n",jointpos(0,0),jointpos(1,0),jointpos(2,0),jointpos(3,0),jointpos(4,0),jointpos(5,0));
    currentp(0,0) = jointpos(0,0);
    currentp(1,0) = jointpos(1,0);
    currentp(2,0) = jointpos(2,0);
    currentp(3,0) = jointpos(3,0);
    currentp(4,0) = jointpos(4,0);
    currentp(5,0) = jointpos(5,0);
    
    //current cartesian velocity
    currentdp = (currentp - prevp)/freq;
    prevp = currentp;   //record current cartesian position for the next step

    //Compute cartersian errors
    pderror = pd - currentp;        //pos error
    dpderror = dpd - currentdp;     //vel error

    errornorm = sqrt(pow(pderror(0,0),2) + pow(pderror(1,0),2) + pow(pderror(2,0),2));
    //Max error recording
    if(fabs(pderror(0,0))>fabs(maxperror(0,0))) maxperror(0,0) = pderror(0,0);  //x
    if(fabs(pderror(1,0))>fabs(maxperror(1,0))) maxperror(1,0) = pderror(1,0);  //y
    if(fabs(pderror(2,0))>fabs(maxperror(2,0))) maxperror(2,0) = pderror(2,0);  //z
    if(fabs(errornorm)>fabs(maxerrornorm)) maxerrornorm = errornorm;  //max error norm



    //Desired joint angle:
    pdesired[0] = pd(0,0);
    pdesired[1] = pd(1,0);
    pdesired[2] = pd(2,0);
    pdesired[3] = pd(3,0);
    pdesired[4] = pd(4,0);
    pdesired[5] = pd(5,0);
    InvK7_1(pdesired, SI_FIX);
    qd(0,0) = q[0];
    qd(1,0) = q[1];
    qd(2,0) = q[2];
    qd(3,0) = q[3];
    qd(4,0) = q[4];
    qd(5,0) = q[5];
    qd(6,0) = q[6];


    //Update the differentiate of Jacobian
    Jacob = Jacobian_Matrix(current_joint_states);
    DiffJacob = DiffJacobian_Matrix(current_joint_states, current_joint_vel);
    Coriolis = Coriolis_Matrix(current_joint_states, current_joint_vel);
    Mass = Mass_Matrix(current_joint_states);
    Gravity = Gravity_Matrix(current_joint_states);
    //Update pseudo-inverse of Jacobian:
    pinv_Jacob = Jacob.transpose()*(Jacob*Jacob.transpose()).inverse();
    Mpinv_Jacob = Mass.inverse()*Jacob.transpose()*(Jacob*Mass.inverse()*Jacob.transpose()).inverse();
    Diff_Mpinv_Jacob = (Mpinv_Jacob - Mpinv_Jacob_Prev)/freq;
    Mpinv_Jacob_Prev = Mpinv_Jacob;
   
    //* 
    ddpd(0,0) = 0;
    ddpd(1,0) = 0;
    ddpd(2,0) = 0;
    ddpd(3,0) = 0;
    ddpd(4,0) = 0;
    ddpd(5,0) = 0;
    //*/

    alpha1 = 1;  //-0.5 -0.6 -0.8 -0.03 -0.04
    alpha2 = 0;

    /* 
    dy1 = TorqueOptCase4(current_joint_states);   //NEW PROPOSED
    //dy1 = dy1_prev + freq*ddy1_prev;
    dy2 = dy2_prev + freq*ddy2_prev;
    dy = alpha1*dy1 + alpha2*dy2;
    
    //Desired joint vel
    dqd = Mpinv_Jacob*dpd + (I7-Mpinv_Jacob*Jacob)*dy;

    ddy1 = -(dy1 - dy1_prev)/freq;
    //ddy1 = TorqueOptCase5(current_joint_states);
    ddy2 = - Mass.inverse()*(Coriolis + Gravity);
    ddy = alpha1*ddy1 + alpha2*ddy2;

    dy1_prev = dy1;
    dy2_prev = dy2;
    ddy2_prev = ddy2;
    ddy1_prev = ddy1;
    */
    dy1 = TorqueOptCase4(current_joint_states);
    dy2 = dy2_prev + freq*ddy2_prev;
    dy = dy2;
    dqd = Mpinv_Jacob*dpd + (I7-Mpinv_Jacob*Jacob)*dy;

    eta1 = (Mass.inverse()*(Gravity)).norm()/((Mass.inverse()*(Coriolis)).norm()+(Mass.inverse()*(Gravity)).norm());
    eta2 = 5*(Mass.inverse()*(Coriolis)).norm()/((Mass.inverse()*(Coriolis)).norm()+(Mass.inverse()*(Gravity)).norm());

    ddy2 = - Mass.inverse()*(Coriolis) - Mass.inverse()*(-eta2*dy1);
    //ddy2 = - Mass.inverse()*(eta1*Coriolis);
    ddy = ddy2;

    dy2_prev = dy2;
    ddy2_prev = ddy2;

    printf("errornorm= %.4f maxerrornorm= %.4f \n",errornorm,maxerrornorm);
    //Null space velocity error
    e_Ndot = (I7 - Mpinv_Jacob*Jacob)*(dqd - current_dq);//   ddy - alpha1*ddy1_prev - alpha2*ddy2_prev;
    //Null space acceleration
    phi_N = (I7 - Mpinv_Jacob*Jacob)*(ddy + 4*e_Ndot) - (Mpinv_Jacob*DiffJacob*Mpinv_Jacob + Diff_Mpinv_Jacob)*Jacob*(dqd - current_dq);   //Diff_Mpinv_Jacob

    //ddqd = pinv_Jacob*(ddpd + kp_matrix_6*pderror + kd_matrix_6*dpderror - DiffJacob*current_dq) - 700*(current_q-qd) - 50*current_dq;
    ddqd = Mpinv_Jacob*(ddpd + kp_matrix_6*pderror + kd_matrix_6*dpderror - DiffJacob*current_dq) + phi_N;


    if(timecounting>=t_total)
    {
        dqd.setZero();
        ddqd.setZero();
        joint_publish(current_joint_states);
    }

    //Compute torque and apply the controller
    Tau_I = Mass*ddqd + Coriolis + Gravity;


    //Torque filter for torque limit avoidance
    for (int i=0; i<7; i++)
    {
        if(Tau_I(i,0) >= T_limit[i]) Tau_I(i,0) = T_limit[i];
        else if (Tau_I(i,0) <= -T_limit[i]) Tau_I(i,0) = -T_limit[i];
    }
    command_tau[0] = Tau_I(0,0);
    command_tau[1] = Tau_I(1,0);
    command_tau[2] = Tau_I(2,0);
    command_tau[3] = Tau_I(3,0);
    command_tau[4] = Tau_I(4,0);
    command_tau[5] = Tau_I(5,0);
    command_tau[6] = Tau_I(6,0);
   
    //Send torque command
    jointt_publish(command_tau);

    //Compute norm torque, efficiency, velocity & acc norm;
    drtorque = drtorque + Tau_I.transpose()*Tau_I*freq;
    taunorm = sqrt(pow(Tau_I(0,0),2) + pow(Tau_I(1,0),2) + pow(Tau_I(2,0),2) + pow(Tau_I(3,0),2) + pow(Tau_I(4,0),2) + pow(Tau_I(5,0),2) + pow(Tau_I(6,0),2));
    sumtaunorm = sumtaunorm + taunorm;
    velocitynorm = current_dq.norm();
    accelerationnorm = current_ddq.norm();
    
}

void inversedynamics_torqueopt2()
{
    //Input desired cartesian pos, vel, acc: pd, dpd, ddpd

    if(savedata<10)
        {
            DataSave(runcount);
            
        }
    //Update current joint angle
    current_q(0,0) = current_joint_states[0];
    current_q(1,0) = current_joint_states[1];
    current_q(2,0) = current_joint_states[2];
    current_q(3,0) = current_joint_states[3];
    current_q(4,0) = current_joint_states[4];
    current_q(5,0) = current_joint_states[5];
    current_q(6,0) = current_joint_states[6];
    //Update current joint angular velocity
    current_dq = (current_q - prev_q)/freq;
    current_ddq = (current_dq - prev_dq)/freq;
    prev_q = current_q;     //record current joint angle for the next step
    prev_dq = current_dq;   //record current joint vel angle for the next step

    current_joint_vel[0] = current_dq(0,0);
    current_joint_vel[1] = current_dq(1,0);
    current_joint_vel[2] = current_dq(2,0);
    current_joint_vel[3] = current_dq(3,0);
    current_joint_vel[4] = current_dq(4,0);
    current_joint_vel[5] = current_dq(5,0);
    current_joint_vel[6] = current_dq(6,0);


    //current cartesian position
    jointpos=forwd7(current_joint_states);
    //Compute current Cartesian state
    printf("x= %.4f y= %.4f z= %.4f alpha= %.4f beta= %.4f gamma= %.4f \n",jointpos(0,0),jointpos(1,0),jointpos(2,0),jointpos(3,0),jointpos(4,0),jointpos(5,0));
    currentp(0,0) = jointpos(0,0);
    currentp(1,0) = jointpos(1,0);
    currentp(2,0) = jointpos(2,0);
    currentp(3,0) = jointpos(3,0);
    currentp(4,0) = jointpos(4,0);
    currentp(5,0) = jointpos(5,0);
    
    //current cartesian velocity
    currentdp = (currentp - prevp)/freq;
    prevp = currentp;   //record current cartesian position for the next step

    //Compute cartersian errors
    pderror = pd - currentp;        //pos error
    dpderror = dpd - currentdp;     //vel error

    errornorm = sqrt(pow(pderror(0,0),2) + pow(pderror(1,0),2) + pow(pderror(2,0),2));
    //Max error recording
    if(fabs(pderror(0,0))>fabs(maxperror(0,0))) maxperror(0,0) = pderror(0,0);  //x
    if(fabs(pderror(1,0))>fabs(maxperror(1,0))) maxperror(1,0) = pderror(1,0);  //y
    if(fabs(pderror(2,0))>fabs(maxperror(2,0))) maxperror(2,0) = pderror(2,0);  //z
    if(fabs(errornorm)>fabs(maxerrornorm)) maxerrornorm = errornorm;  //max error norm



    //Desired joint angle:
    pdesired[0] = pd(0,0);
    pdesired[1] = pd(1,0);
    pdesired[2] = pd(2,0);
    pdesired[3] = pd(3,0);
    pdesired[4] = pd(4,0);
    pdesired[5] = pd(5,0);
    InvK7_1(pdesired, SI_FIX);
    qd(0,0) = q[0];
    qd(1,0) = q[1];
    qd(2,0) = q[2];
    qd(3,0) = q[3];
    qd(4,0) = q[4];
    qd(5,0) = q[5];
    qd(6,0) = q[6];


    //Update the differentiate of Jacobian
    Jacob = Jacobian_Matrix(current_joint_states);
    DiffJacob = DiffJacobian_Matrix(current_joint_states, current_joint_vel);
    Coriolis = Coriolis_Matrix(current_joint_states, current_joint_vel);
    Mass = Mass_Matrix(current_joint_states);
    Gravity = Gravity_Matrix(current_joint_states);
    //Update pseudo-inverse of Jacobian:
    pinv_Jacob = Jacob.transpose()*(Jacob*Jacob.transpose()).inverse();
    Mpinv_Jacob = Mass.inverse()*Jacob.transpose()*(Jacob*Mass.inverse()*Jacob.transpose()).inverse();
    Diff_Mpinv_Jacob = (Mpinv_Jacob - Mpinv_Jacob_Prev)/freq;
    Mpinv_Jacob_Prev = Mpinv_Jacob;
   
    //* 
    ddpd(0,0) = 0;
    ddpd(1,0) = 0;
    ddpd(2,0) = 0;
    ddpd(3,0) = 0;
    ddpd(4,0) = 0;
    ddpd(5,0) = 0;
    //*/

    alpha1 = 1;  //-0.5 -0.6 -0.8 -0.03 -0.04
    alpha2 = 0;

    /* 
    dy1 = TorqueOptCase4(current_joint_states);   //NEW PROPOSED
    //dy1 = dy1_prev + freq*ddy1_prev;
    dy2 = dy2_prev + freq*ddy2_prev;
    dy = alpha1*dy1 + alpha2*dy2;
    
    //Desired joint vel
    dqd = Mpinv_Jacob*dpd + (I7-Mpinv_Jacob*Jacob)*dy;

    ddy1 = -(dy1 - dy1_prev)/freq;
    //ddy1 = TorqueOptCase5(current_joint_states);
    ddy2 = - Mass.inverse()*(Coriolis + Gravity);
    ddy = alpha1*ddy1 + alpha2*ddy2;

    dy1_prev = dy1;
    dy2_prev = dy2;
    ddy2_prev = ddy2;
    ddy1_prev = ddy1;
    */
    //Desired joint acceleration

    dy1 = TorqueOptCase4(current_joint_states);
    dy2 = dy2_prev + freq*ddy2_prev;
    dy = dy2;
    dqd = Mpinv_Jacob*dpd;// + (I7-Mpinv_Jacob*Jacob)*dy;

    eta1 = (Mass.inverse()*(Gravity)).norm()/((Mass.inverse()*(Coriolis)).norm()+(Mass.inverse()*(Gravity)).norm());
    eta2 = 20*(Mass.inverse()*(Coriolis)).norm()/((Mass.inverse()*(Coriolis)).norm()+(Mass.inverse()*(Gravity)).norm());

    ddy2 = - Mass.inverse()*(Coriolis);// - Mass.inverse()*(-eta2*dy1);
    //ddy2 = - Mass.inverse()*(eta1*Coriolis);
    ddy = ddy2;

    dy2_prev = dy2;
    ddy2_prev = ddy2;

    printf("errornorm= %.4f maxerrornorm= %.4f \n",errornorm,maxerrornorm);
    //Null space velocity error
    e_Ndot = (I7 - Mpinv_Jacob*Jacob)*(dqd - current_dq);//   ddy - alpha1*ddy1_prev - alpha2*ddy2_prev;
    //Null space acceleration
    phi_N = (I7 - Mpinv_Jacob*Jacob)*(ddy + 4*e_Ndot) - (Mpinv_Jacob*DiffJacob*Mpinv_Jacob + Diff_Mpinv_Jacob)*Jacob*(dqd - current_dq);   //Diff_Mpinv_Jacob

    //ddqd = pinv_Jacob*(ddpd + kp_matrix_6*pderror + kd_matrix_6*dpderror - DiffJacob*current_dq) - 700*(current_q-qd) - 50*current_dq;
    ddqd = Mpinv_Jacob*(ddpd + kp_matrix_6*pderror + kd_matrix_6*dpderror - DiffJacob*current_dq);// + phi_N;


    //Compute torque and apply the controller
    Tau_I = Mass*ddqd + Coriolis + Gravity;
    

    //Torque filter for torque limit avoidance
    for (int i=0; i<7; i++)
    {
        if(Tau_I(i,0) >= T_limit[i]) Tau_I(i,0) = T_limit[i];
        else if (Tau_I(i,0) <= -T_limit[i]) Tau_I(i,0) = -T_limit[i];
    }
    command_tau[0] = Tau_I(0,0);
    command_tau[1] = Tau_I(1,0);
    command_tau[2] = Tau_I(2,0);
    command_tau[3] = Tau_I(3,0);
    command_tau[4] = Tau_I(4,0);
    command_tau[5] = Tau_I(5,0);
    command_tau[6] = Tau_I(6,0);
   
    //Send torque command
    jointt_publish(command_tau);

    //Compute norm torque, efficiency, velocity & acc norm;
    drtorque = drtorque + Tau_I.transpose()*Tau_I*freq;
    taunorm = sqrt(pow(Tau_I(0,0),2) + pow(Tau_I(1,0),2) + pow(Tau_I(2,0),2) + pow(Tau_I(3,0),2) + pow(Tau_I(4,0),2) + pow(Tau_I(5,0),2) + pow(Tau_I(6,0),2));
    sumtaunorm = sumtaunorm + taunorm;
    velocitynorm = current_dq.norm();
    accelerationnorm = current_ddq.norm();
}

void cartesianstepcomputing()
{
    

    linear_disp = sqrt(pow((pend[0]-pinit[0]),2)+pow((pend[1]-pinit[1]),2)+pow((pend[2]-pinit[2]),2));
    angular_disp = sqrt(pow((pend[3]-pinit[3]),2)+pow((pend[4]-pinit[4]),2)+pow((pend[5]-pinit[5]),2));
    if (linear_disp == 0) linear_disp = 0.000001;
    if (angular_disp == 0) angular_disp = 0.000001;
    
    linear_dir_vec[0] = (pend[0] - pinit[0]) / linear_disp;
	linear_dir_vec[1] = (pend[1] - pinit[1]) / linear_disp;
	linear_dir_vec[2] = (pend[2] - pinit[2]) / linear_disp;

    
    angular_dir_vec[0] = (pend[3] - pinit[3]) / angular_disp;
	angular_dir_vec[1] = (pend[4] - pinit[4]) / angular_disp;
	angular_dir_vec[2] = (pend[5] - pinit[5]) / angular_disp;

    //Trapezoid velocity curve params - divided into 7 segments
    //V1(t)=linear_a1*t^2+linear_b1*t+linear_c1
    //V2(t)=linear_a2*t+linear_b2
    //V3(t)=linear_a3*t^2+linear_b3*t+linear_c3
    //V4(t)=vmax
    //V5(t)=linear_a5*t^2+linear_b5*t+linear_c5
    //V6(t)=linear_a6*t+linear_b6
    //V7(t)=linear_a7*t^2+linear_b7*t+linear_c7


    //Linear movements params
    linear_a1 = (pow(N_portion,2)*linear_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    linear_b1 = 0;
    linear_c1 = 0;

    linear_a2 = (N_portion*linear_disp)/((N_portion-1)*t_acc*(t_total-t_acc));
    linear_b2 = -linear_disp/(2*(N_portion-1)*(t_total-t_acc));
    linear_c2 = (t_acc*linear_disp)/(6*(N_portion-1)*N_portion*(t_total-t_acc));

    linear_a3 = -(pow(N_portion,2)*linear_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    linear_b3 = (pow(N_portion,2)*linear_disp)/((N_portion-1)*t_acc*(t_total-t_acc));
    linear_c3 = -((pow(N_portion,2)-2*N_portion+2)*linear_disp)/(2*(N_portion-1)*(t_total-t_acc));

    linear_vmax = linear_disp/(t_total-t_acc);

    linear_a5 = -(pow(N_portion,2)*linear_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    linear_b5 = (pow(N_portion,2)*linear_disp)/((N_portion-1)*pow(t_acc,2));
    linear_c5 = ((-pow(N_portion,2)*pow(t_total,2)+2*pow(N_portion,2)*t_total*t_acc-(pow(N_portion,2)-2*N_portion+2)*pow(t_acc,2))*linear_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));

    linear_a6 = -(N_portion*linear_disp)/((N_portion-1)*t_acc*(t_total-t_acc));
    linear_b6 = ((2*N_portion*t_total-t_acc)*linear_disp)/(2*(N_portion-1)*t_acc*(t_total-t_acc));

    linear_a7 = (pow(N_portion,2)*linear_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    linear_b7 = -(pow(N_portion,2)*t_total*linear_disp)/((N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    linear_c7 = (pow(N_portion,2)*pow(t_total,2)*linear_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
     
    //Angular movements params
    angular_a1 = (pow(N_portion,2)*angular_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    angular_b1 = 0;
    angular_c1 = 0;

    angular_a2 = (N_portion*angular_disp)/((N_portion-1)*t_acc*(t_total-t_acc));
    angular_b2 = -angular_disp/(2*(N_portion-1)*(t_total-t_acc));
    angular_c2 = (t_acc*angular_disp)/(6*(N_portion-1)*N_portion*(t_total-t_acc));

    angular_a3 = -(pow(N_portion,2)*angular_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    angular_b3 = (pow(N_portion,2)*angular_disp)/((N_portion-1)*t_acc*(t_total-t_acc));
    angular_c3 = -((pow(N_portion,2)-2*N_portion+2)*angular_disp)/(2*(N_portion-1)*(t_total-t_acc));

    angular_vmax = angular_disp/(t_total-t_acc);

    angular_a5 = -(pow(N_portion,2)*angular_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    angular_b5 = (pow(N_portion,2)*angular_disp)/((N_portion-1)*pow(t_acc,2));
    angular_c5 = ((-pow(N_portion,2)*pow(t_total,2)+2*pow(N_portion,2)*t_total*t_acc-(pow(N_portion,2)-2*N_portion+2)*pow(t_acc,2))*angular_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));

    angular_a6 = -(N_portion*angular_disp)/((N_portion-1)*t_acc*(t_total-t_acc));
    angular_b6 = ((2*N_portion*t_total-t_acc)*angular_disp)/(2*(N_portion-1)*t_acc*(t_total-t_acc));

    angular_a7 = (pow(N_portion,2)*angular_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    angular_b7 = -(pow(N_portion,2)*t_total*angular_disp)/((N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
    angular_c7 = (pow(N_portion,2)*pow(t_total,2)*angular_disp)/(2*(N_portion-1)*pow(t_acc,2)*(t_total-t_acc));
}
//*********************************Handle Signal for Stop********************************************
void signal_handler(int signum = 0)
{
	
	printf("Simulation is stopped!\n");
	exit(1);
}





//****************************************MAIN PROGRAM************************************************
int main(int argc, char **argv)

{
	std::string sep = "\n----------------------------------------\n";
	ros::init(argc, argv, "torque_downscale");  // Node name initialization
	ros::NodeHandle nh;                            // Node handle declaration for communication with ROS

	pub_jointp = nh.advertise<downscale4::seven>("gazebo/downscale_jointp", 100);
	pub_jointt = nh.advertise<downscale4::seven>("gazebo/downscale_jointt", 100);

    ros::Subscriber hd_trans = nh.subscribe("/hd_trans", 1, &hd_callback_trans);        //sub transformation matrix
	ros::Subscriber hd_buttons = nh.subscribe("/hd_buttons", 1, &hd_callback_buttons);  //sub phantom omni buttons
	ros::Subscriber hd_char = nh.subscribe("/hd_char", 1, &hd_callback_keyboards); 
    ros::Subscriber sub_jointp = nh.subscribe("downscale_actp", 100, msgCallbackP);     //sub joint position
	ros::Subscriber sub_jointt = nh.subscribe("downscale_actt", 100, msgCallbackT);     //sub joint torque
	ros::Subscriber sub_jointv = nh.subscribe("downscale_actv", 100, msgCallbackv);     //sub joint velocity
    

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	
    ros::Rate rate(1000.); // loop frequency[Hz]
	ros::spinOnce();

    remove("MKE.txt");

	//Torque limits
	T_limit[0] = 200;
	T_limit[1] = 400;
	T_limit[2] = 350;
	T_limit[3] = 350;
	T_limit[4] = 200;
	T_limit[5] = 300;
	T_limit[6] = 70;

    while(current_joint_states[0]==0)
	{
		joint_publish(q);
		ros::spinOnce();
		printf(" waiting for getting  value from gazebo...\n");
	}
    printf(" waiting 0.001 sec...\n");
    joint_publish(q);

    
	ros::Duration(freq).sleep();


    //Given desired damper matrix 
	kd_matrix_7 << 	90, 0,	0,0,0,0,0,
                     0, 100, 0,0,0,0,0,	
                     0, 0, 90,0,0,0,0,
                     0, 0, 	0,70,0,0,0,
                     0, 0, 	0,0,20,0,0,
                     0, 0, 	0,0,0,20,0, 
                     0, 0, 	0,0,0,0,15;

    //Given desired spring matrix 
    
	kp_matrix_7 << 	3000, 0, 0,0,0,0,0,
                     0, 3200, 0,0,0,0,0,
                     0, 0, 3200,0,0,0,0,
                     0, 0, 	0,2500,0,0,0,
                     0, 0, 	0,0,1800,0,0,
                     0, 0, 	0,0,0,1000,0,
                     0, 0, 	0,0,0,0,500;

    //Given desired spring matrix 
    //* 
    kp_matrix_6 << 	 4500, 0, 0,0,0,0,
                    0, 4000, 0,0,0,0,
                    0, 0, 4000,0,0,0,
                    0, 0, 	0,0,0,0,
                    0, 0, 	0,0,0,0,
                    0, 0, 	0,0,0,0; 

    kd_matrix_6 << 	37, 0,	0,0,0,0,
                    0, 35,   0,0,0,0,	
                    0, 0,   35,0,0,0,
                    0, 0, 	0,0,0,0,
                    0, 0, 	0,0,0,0,
                    0, 0, 	0,0,0,0;

    
    I7 = I7.setIdentity(7,7);
    I6 = I6.setIdentity(6,6);

 
    double readtheta[30000][7];
    std::ifstream input("LNpp20srefine.txt");   //LNpp20srefine     LNmke20srefine1_notfall     QCpp25srefine1      QCpp25smke1
  
    qdprev = qd;
    int counttheta=0;

    //Getting joint space trajectories data
    for(std::string line; getline(input, line);)
        {
            //input >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5] >> q[6];
            input >> readtheta[counttheta][0] >> readtheta[counttheta][1] >> readtheta[counttheta][2] >> readtheta[counttheta][3] >> readtheta[counttheta][4] >> readtheta[counttheta][5] >> readtheta[counttheta][6];
            counttheta++;
        }
    /* 
    for(int i=0; i<counttheta-1; i++)
    {
        q[0] = readtheta[i][0];
        q[1] = readtheta[i][1];
        q[2] = readtheta[i][2];
        q[3] = readtheta[i][3];
        q[4] = readtheta[i][4];
        q[5] = readtheta[i][5];
        q[6] = readtheta[i][6];
        printf("q0= %.4f q1= %.4f q2= %.4f q3= %.4f q4= %.4f q5= %.4f q6= %.4f \n",q[0],q[1],q[2],q[3],q[4],q[5],q[6]);   
    }
    */
    int countread=0;

    //Move arm to init configuration
    for(int k=0; k<500; k++)
    {
        q[0] = readtheta[countread][0];
        q[1] = readtheta[countread][1];
        q[2] = readtheta[countread][2];
        q[3] = readtheta[countread][3];
        q[4] = readtheta[countread][4];
        q[5] = readtheta[countread][5];
        q[6] = readtheta[countread][6];
        
        qd(0,0) = q[0];
        qd(1,0) = q[1];
        qd(2,0) = q[2];
        qd(3,0) = q[3];
        qd(4,0) = q[4];
        qd(5,0) = q[5];
        qd(6,0) = q[6];
        
        dqd = (qd - qdprev)/freq;
        qdprev = qd;

        //Update current joint angle
        current_q(0,0) = current_joint_states[0];
        current_q(1,0) = current_joint_states[1];
        current_q(2,0) = current_joint_states[2];
        current_q(3,0) = current_joint_states[3];
        current_q(4,0) = current_joint_states[4];
        current_q(5,0) = current_joint_states[5];
        current_q(6,0) = current_joint_states[6];
        //Update current joint angular velocity
        current_dq = (current_q - prev_q)/freq;

        prev_q = current_q;     //record current joint angle for the next step

        //Update the differentiate of Jacobian
        Jacob = Jacobian_Matrix(current_joint_states);
        DiffJacob = DiffJacobian_Matrix(current_joint_states, current_joint_vel);
        Coriolis = Coriolis_Matrix(current_joint_states, current_joint_vel);
        Mass = Mass_Matrix(current_joint_states);
        Gravity = Gravity_Matrix(current_joint_states);
        //Update pseudo-inverse of Jacobian:
        pinv_Jacob = Jacob.transpose()*(Jacob*Jacob.transpose()).inverse();
        Mpinv_Jacob = Mass.inverse()*Jacob.transpose()*(Jacob*Mass.inverse()*Jacob.transpose()).inverse();

        //kd_matrix_7 = Mpinv_Jacob*kd_matrix_6*Jacob;
        //kp_matrix_7 = Mpinv_Jacob*kp_matrix_6*Jacob;

        dqdupt = qd - current_q;
        //Compute torque and apply the controller
        Tau_I =  kp_matrix_7*(qd - current_q) + kd_matrix_7*(dqd - current_dq) + Coriolis + Gravity;
        
        //Torque filter for torque limit avoidance
        for (int i=0; i<7; i++)
        {
            if(Tau_I(i,0) >= T_limit[i]) Tau_I(i,0) = T_limit[i];
            else if (Tau_I(i,0) <= -T_limit[i]) Tau_I(i,0) = -T_limit[i];
        }
        command_tau[0] = Tau_I(0,0);
        command_tau[1] = Tau_I(1,0);
        command_tau[2] = Tau_I(2,0);
        command_tau[3] = Tau_I(3,0);
        command_tau[4] = Tau_I(4,0);
        command_tau[5] = Tau_I(5,0);
        command_tau[6] = Tau_I(6,0);

        //Send torque command
        //jointt_publish(command_tau);
        
        joint_publish(q);

		ros::spinOnce();
        ros::Duration(freq).sleep();
    }
	while (ros::ok())
	{  	   
        if(savedata<5)
        {
            DataSave(runcount);
        }

        q[0] = readtheta[countread][0];
        q[1] = readtheta[countread][1];
        q[2] = readtheta[countread][2];
        q[3] = readtheta[countread][3];
        q[4] = readtheta[countread][4];
        q[5] = readtheta[countread][5];
        q[6] = readtheta[countread][6];
        //printf("q0= %.4f q1= %.4f q2= %.4f q3= %.4f q4= %.4f q5= %.4f q6= %.4f \n",q[0],q[1],q[2],q[3],q[4],q[5],q[6]);

        qd(0,0) = q[0];
        qd(1,0) = q[1];
        qd(2,0) = q[2];
        qd(3,0) = q[3];
        qd(4,0) = q[4];
        qd(5,0) = q[5];
        qd(6,0) = q[6];
        
        dqd = (qd - qdprev)/freq;
        qdprev = qd;

        //Update current joint angle
        current_q(0,0) = current_joint_states[0];
        current_q(1,0) = current_joint_states[1];
        current_q(2,0) = current_joint_states[2];
        current_q(3,0) = current_joint_states[3];
        current_q(4,0) = current_joint_states[4];
        current_q(5,0) = current_joint_states[5];
        current_q(6,0) = current_joint_states[6];
        //Update current joint angular velocity
        current_dq = (current_q - prev_q)/freq;

        prev_q = current_q;     //record current joint angle for the next step
        prev_dq = current_dq;   //record current joint vel angle for the next step

        //Compute current Cartesian state
        jointpos=forwd7(current_joint_states);
        //printf("x= %.4f y= %.4f z= %.4f alpha= %.4f beta= %.4f gamma= %.4f \n",jointpos(0,0),jointpos(1,0),jointpos(2,0),jointpos(3,0),jointpos(4,0),jointpos(5,0));

        //Update the differentiate of Jacobian
        Jacob = Jacobian_Matrix(current_joint_states);
        DiffJacob = DiffJacobian_Matrix(current_joint_states, current_joint_vel);
        Coriolis = Coriolis_Matrix(current_joint_states, current_joint_vel);
        Mass = Mass_Matrix(current_joint_states);
        Gravity = Gravity_Matrix(current_joint_states);
        //Update pseudo-inverse of Jacobian:
        pinv_Jacob = Jacob.transpose()*(Jacob*Jacob.transpose()).inverse();
        Mpinv_Jacob = Mass.inverse()*Jacob.transpose()*(Jacob*Mass.inverse()*Jacob.transpose()).inverse();

        //kd_matrix_7 = Mpinv_Jacob*kd_matrix_6*Jacob;
        //kp_matrix_7 = Mpinv_Jacob*kp_matrix_6*Jacob;

        dqdupt = qd - current_q;
        //Compute torque and apply the controller
        Tau_I =  kp_matrix_7*(qd - current_q) + kd_matrix_7*(dqd - current_dq) + Coriolis + Gravity;
        printf("T1= %.4f; T2= %.4f; T3= %.4f; T4= %.4f; T5= %.4f; T6 %.4f; T7= %.4f \n",Tau_I(0,0),Tau_I(1,0),Tau_I(2,0),Tau_I(3,0),Tau_I(4,0),Tau_I(5,0),Tau_I(6,0));
        //printf("q0= %.4f q1= %.4f q2= %.4f q3= %.4f q4= %.4f q5= %.4f q6= %.4f \n",dqdupt(0,0),dqdupt(1,0),dqdupt(2,0),dqdupt(3,0),dqdupt(4,0),dqdupt(5,0),dqdupt(6,0));
        
        //Torque filter for torque limit avoidance
        for (int i=0; i<7; i++)
        {
            if(Tau_I(i,0) >= T_limit[i]) Tau_I(i,0) = T_limit[i];
            else if (Tau_I(i,0) <= -T_limit[i]) Tau_I(i,0) = -T_limit[i];
        }
        command_tau[0] = Tau_I(0,0);
        command_tau[1] = Tau_I(1,0);
        command_tau[2] = Tau_I(2,0);
        command_tau[3] = Tau_I(3,0);
        command_tau[4] = Tau_I(4,0);
        command_tau[5] = Tau_I(5,0);
        command_tau[6] = Tau_I(6,0);

        //Send torque command
        jointt_publish(command_tau);
        
        drtorque = drtorque + Tau_I.transpose()*Tau_I*freq;
        taunorm = sqrt(pow(Tau_I(0,0),2) + pow(Tau_I(1,0),2) + pow(Tau_I(2,0),2) + pow(Tau_I(3,0),2) + pow(Tau_I(4,0),2) + pow(Tau_I(5,0),2) + pow(Tau_I(6,0),2));
        sumtaunorm = sumtaunorm + taunorm;

        

        if(countread==counttheta-2) {countread=countread; savedata++;}
        else
        {
            countread++;
        }
        runcount++;
        ros::spinOnce();
        ros::Duration(freq).sleep();  
    } //end while main
	//myfile.close();
	return 0;	
    
} //end main
