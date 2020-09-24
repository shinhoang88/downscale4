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
#include "ros/ros.h"
#include "ros/time.h"
#include "scrlib.h"


bool	KeyMode(ros::NodeHandle);
bool	ScenarioMode(void);

//ArmTrajectory A_Traj;
AKfun AKin;
using namespace std;

double fx;

void Initialize()
{
  

}


int main(int argc, char **argv)
{

  // Set up ROS.
  ros::init(argc, argv, "MainControl");
  ros::NodeHandle nh; 

  Initialize();

  float rate = 125.0;			//frequency of the loop
  ros::Rate loop_rate(rate);

  char	key_MODE='0';
  fx=0;
  while(nh.ok())
  {
    scr_clear();
    scr_move(0, 4);
    printf("MODE SELECT: Select Your Task                                       \n");
    printf("1 : Key Mode,  2 : Scenario Mode,  q : Quit    \n");

    key_MODE=getchar();
		
    switch(key_MODE)
    {	
      case '1':
        while(KeyMode(nh)){
        }
      break;

      case '2':
        while(ScenarioMode()){
        }
      break;
    }

    if (key_MODE=='q'){
      break;
    }

    loop_rate.sleep();
  }

  return 0;
}


bool KeyMode(ros::NodeHandle nh)
{

  //ros::Publisher main_pub = nh.advertise<skku_handarm::Armmsg>("MainCommand_msg", 100);
  ros::Publisher Cmd2Arm_pub = nh.advertise<skku_handarm::Main2Arm>("MainCommand", 1);
  ros::Publisher pub = nh.advertise<ur10_gazebo::ur10>("ur_driver/joint_cmd", 100);


	char	key_MODE[3];
	int i=0, j=0, x=0, y=0, Return_Flag=0;
	static float ArmPosD[6]={0,0,0,0,0,0};
	static float ArmPosA[6]={0,0,0,0,0,0};
  	ros::Rate loop_rat(250);

    VectorXd Arm_qc(6);
	VectorXd Arm_qd(6);

    scr_move(0, 4);
	printf("KEY MODE : Select Your Task                                           \n");
	printf(" 0-20 : Faraman Position, 6-0 : Hand Position q : Quit                    \n");
  //float pi=3.14159265359;

	std::vector<int> FingerCmd(4);
	ros::param::set("/SkkuHandCmd",FingerCmd);
	for(int j=0;j<4;j++){
		FingerCmd[j]=1;
	}

	fgets(key_MODE, 3, stdin);


			Matrix4f	testPosd;


	if (key_MODE[0] == 'q'){
		return false;
	}
	  else if (key_MODE[0] == 'w'){
    std::vector<double> cmd;
    cmd.push_back(2);
    cmd.push_back(0);
    cmd.push_back(-0.8);
    cmd.push_back(-1.57);
    cmd.push_back(0);
    cmd.push_back(0);
    cmd.push_back(0);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	/*else if (key_MODE[0] == 'b'){
    std::vector<double> cmd;
    cmd.push_back(2);
    cmd.push_back(0);
    cmd.push_back(-1.04719755);
    cmd.push_back(-2.0943295);
    cmd.push_back(0);
    cmd.push_back(1.57);
    cmd.push_back(0);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'e'){
    std::vector<double> cmd;
   //cmd.push_back(71);
    cmd.push_back(2);
    cmd.push_back(0.43633);
    cmd.push_back(-2.02807259);
    cmd.push_back(-1.816538685);
    cmd.push_back(0.7030186);
    cmd.push_back(1.134464);
    cmd.push_back(0);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'r'){
    std::vector<double> cmd;
    //cmd.push_back(72);
    cmd.push_back(2);
    cmd.push_back(0.08726646);
    cmd.push_back(-1.813571626);
    cmd.push_back(-1.494176373);
    cmd.push_back(0.166155344);
    cmd.push_back(1.483529864);
    cmd.push_back(0);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}*/
	/*else if (key_MODE[0] == 'r'){
    std::vector<double> cmd;
    //cmd.push_back(72);
    cmd.push_back(2);
    cmd.push_back(-1.44468);
    cmd.push_back(-1.92644);
    cmd.push_back(-2.30025);
    cmd.push_back(1.07);
    cmd.push_back(1.54783);
    cmd.push_back(-0.0113632);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'e'){
    std::vector<double> cmd;
   //cmd.push_back(71);
    cmd.push_back(2);
    cmd.push_back(0.481703);
    cmd.push_back(-1.84018);
    cmd.push_back(-1.97908);
    cmd.push_back(-0.881069);
    cmd.push_back(1.48021);
    cmd.push_back(1.27722);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'w'){
    std::vector<double> cmd;
    cmd.push_back(2);
    cmd.push_back(-0.31849);
    cmd.push_back(-2.4445);
    cmd.push_back(-1.90951);
    cmd.push_back(1.18522);
    cmd.push_back(1.11727);
    cmd.push_back(0.0497169);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}*/
////////////////////////////////////////////////////////////////////////////////calibration with UR10 using sherical helix path
/*	else if (key_MODE[0] == 'a'){
		std::vector<double> cmd;
		cmd.push_back(98);
    cmd.push_back(-0.7854);
    cmd.push_back(-3.34);
    cmd.push_back(1.37);
    cmd.push_back(-2.74);
    cmd.push_back(-1.57);
    cmd.push_back(-0.7854);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'b'){
		std::vector<double> cmd;
		cmd.push_back(99);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'c'){
		std::vector<double> cmd;
		cmd.push_back(98);
    cmd.push_back(-0.7854);
    cmd.push_back(-2.1817);
    cmd.push_back(-1.5708);
    cmd.push_back(-1.0472);
    cmd.push_back(1.5708);
    cmd.push_back(-0.7854);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'd'){
		std::vector<double> cmd;
		cmd.push_back(100);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}*/
////////////////////////////////////////////////////////////////////////////////////////////////
	else if (key_MODE[0] == 'p'){
    std::vector<double> cmd;
    cmd.push_back(0);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	/*else if (key_MODE[0] == 'l'){ 
    std::vector<double> cmd;
    cmd.push_back(8);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if(key_MODE[0] == 'k'){
		std::vector<double> cmd;
    cmd.push_back(9);
		nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'j'){
    std::vector<double> cmd;
    cmd.push_back(34);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'i'){
    std::vector<double> cmd;
    cmd.push_back(35);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}*/
	else if (key_MODE[0] == 's'){
    std::vector<double> cmd;
    cmd.push_back(79);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'f'){
    std::vector<double> cmd;
    cmd.push_back(5);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	/*else if (key_MODE[0] == 'b'){
    std::vector<double> cmd;
    cmd.push_back(11);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'd'){
    std::vector<double> cmd;
    cmd.push_back(6);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'c'){
    std::vector<double> cmd;
    cmd.push_back(75);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}*/
	else if (key_MODE[0] == 'v'){
    std::vector<double> cmd;
    cmd.push_back(8);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'h'){
    std::vector<double> cmd;
    cmd.push_back(10);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	
	/*else if (key_MODE[0] == 'r'){
    std::vector<double> cmd;
    cmd.push_back(91);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}*/
	else if (key_MODE[0] == 't'){
    std::vector<double> cmd;
    cmd.push_back(92);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'y'){
    std::vector<double> cmd;
    cmd.push_back(93);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'u'){
    std::vector<double> cmd;
    cmd.push_back(94);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'n'){
    std::vector<double> cmd;
    cmd.push_back(101);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'm'){
    std::vector<double> cmd;
    cmd.push_back(102);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'l'){
    std::vector<double> cmd;
    cmd.push_back(122);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'k'){
    std::vector<double> cmd;
    cmd.push_back(123);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else if (key_MODE[0] == 'j'){
    std::vector<double> cmd;
    cmd.push_back(124);
    nh.setParam("skku_handarm/ArmCmd",cmd);
	}
	else{
		i = atoi(key_MODE);
		if (i > 0 && i < 15){
			std::vector<double> cmd;
      cmd.push_back(1);
		  for(int x=0;x<3;x++)
        cmd.push_back(URposition[i][x]);
      nh.setParam("skku_handarm/ArmCmd",cmd);

		}
	}

	return true;
}


bool ScenarioMode(void)
{
	printf("ScenarioMode \n");
	return false;
}