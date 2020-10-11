#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define CONTROL_CYCLE 10 //[Hz]
#define LENGTH 0.75	// chassis length
#define WIDTH 0.75	// chassis width
#define HEIGHT 0.25	// chassis height
#define RADIUS 0.2	// wheel radius
#define STARTZ 0.5	// starting height of chassis
#define CMASS 1.0		// chassis mass
#define WMASS 0.5	  // wheel mass

static double velocityIn[3];

//３次元直交座標の定義
struct Cartesian3 {
	double x;
	double y;
	double z;
};
//４輪の車輪位置の設定
struct Cartesian3 w[] =
{
	{	0.5*LENGTH,	0.5*WIDTH,	-0.5*HEIGHT	},//n=0:左前輪
	{	0.5*LENGTH,-0.5*WIDTH,	-0.5*HEIGHT	},//n=1:右前輪
	{  -0.5*LENGTH,	0.5*WIDTH,	-0.5*HEIGHT	},//n=2:左後輪
	{  -0.5*LENGTH,-0.5*WIDTH,	-0.5*HEIGHT	} //n=3:右後輪
};
std::vector<std::string> strList =
{
    "/omnidirectional_four_wheeled_robot/FR_delta/command",
    "/omnidirectional_four_wheeled_robot/RR_delta/command",
    "/omnidirectional_four_wheeled_robot/FL_delta/command",
    "/omnidirectional_four_wheeled_robot/RL_delta/command",
    "/omnidirectional_four_wheeled_robot/FR_omega/command",
    "/omnidirectional_four_wheeled_robot/RR_omega/command",
    "/omnidirectional_four_wheeled_robot/FL_omega/command",
    "/omnidirectional_four_wheeled_robot/RL_omega/command",
};
void velocityCallback(const geometry_msgs::Twist& msgIn){
  velocityIn[0] = msgIn.linear.x;
	velocityIn[1] = msgIn.linear.y;
	velocityIn[2] = msgIn.angular.z;
}
static void ControllerKinematics(double delta[4], double omega[4], double Ux, double Uy, double Uq)
{
  double vx, vy, delta_old;
	for(int n = 0; n < 4; n++){
		vx = Ux - w[n].y * Uq;
		vy = Uy + w[n].x * Uq;
    //前回の操舵角
		delta_old = delta[n];
		//操舵角度を計算する
		if (vx*vx + vy*vy > 1.0E-5){
			delta[n] = atan2(vy, vx);
		}
    //操舵角が±90°を超えて変化
		while (delta[n] >= delta_old + M_PI / 2)	delta[n] -= M_PI;
    //したら180°を増減して調整
		while (delta[n] <= delta_old - M_PI / 2)	delta[n] += M_PI;
		//車輪回転角速度を計算する
		omega[n] = (vx * cos(delta[n]) + vy * sin(delta[n])) / RADIUS;
	}
}
int main(int argc, char **argv)
{
  int i;
  double Ux = 0., Uy = 0., Uq = 0.;
  double delta[4] = {0., 0., 0., 0.};// 目標操舵角度 [rad]
  double omega[4] = {0., 0., 0., 0.};// 目標車輪回転角速度 [rad/s]
  double NOW_TIME;
  double BEGIN_TIME;
  const int pubNum = strList.size();

	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/cmd_vel", 1, velocityCallback);
  ros::Publisher  pub[pubNum];
  std_msgs::Float64 msg[pubNum];

  for(i = 0; i < pubNum; i++) pub[i] = n.advertise<std_msgs::Float64>(strList[i], 1);

  ros::Rate loop_rate(CONTROL_CYCLE);
	BEGIN_TIME = ros::Time::now().toSec();
	while(ros::ok()){
    ros::spinOnce();
		NOW_TIME = ros::Time::now().toSec() - BEGIN_TIME;
		Ux = velocityIn[0];
		Uy = velocityIn[1];
		Uq = velocityIn[2];

		ControllerKinematics(delta, omega, Ux, Uy, Uq);

    for(i = 0; i < 4; i++) msg[i].data = delta[i];
    for(i = 4; i < 8; i++) msg[i].data = omega[i - 4];
    for(i = 0; i < pubNum; i++) pub[i].publish(msg[i]);

    loop_rate.sleep();
  }
  return 0;
}
