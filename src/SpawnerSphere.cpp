#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SpawnerSphere");
	ros::NodeHandle n;

	geometry_msgs::Pose pose;
	gazebo_msgs::SpawnModel model;
	ros::ServiceClient spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

	std::ifstream ifs;
	ifs.open("/home/koji/catkin_ws/src/omnidirectional_four-wheeled_robot/urdf/sphere.urdf");
	ifs >> model.request.model_xml;
	model.request.model_name = "/sphere";
	model.request.initial_pose.position.x = 3.;
	model.request.initial_pose.position.y = 3.;
	model.request.initial_pose.position.z = 3.;
	model.request.initial_pose.orientation.x = 0.;
	model.request.initial_pose.orientation.y = 0.;
	model.request.initial_pose.orientation.z = 0.;
	model.request.initial_pose.orientation.w = 1.;
	model.request.reference_frame = "";
	spawn.call(model);
  return 0;
}
