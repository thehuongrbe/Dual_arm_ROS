#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
//MOVE IT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//visual tool
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
//
float positionX = -0.155026881889;
float positionY = -0.00380711320647;
float positionZ = -0.580344179492;

float orientationX =  0.712362483218;
float orientationY = 0.701773892725;
float orientationZ = 0.00167994660707;
float orientationW = 0.00709040002731 ;
std::string select_group="dual_arm";
std::string link_end ="l_LEnd";
void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	float new_positionX = msg->position.x;
	float new_positionY = msg->position.y;
	float new_positionZ = msg->position.z;

	ROS_INFO("saved_x: [%.9f], new:[%.9f]", positionX, new_positionX);
	ROS_INFO("saved_y: [%.9f], new [%.9f]", positionY, new_positionY);
	ROS_INFO("saved_z: [%.9f], new [%.9f]", positionZ, new_positionZ);
 	

	if ((new_positionX!=positionX )|| (new_positionY!=positionY)||(new_positionZ!=positionZ))
	{

	ROS_INFO("new pose");

	//planning group that we would like to control
	moveit::planning_interface::MoveGroupInterface group(select_group);
	

	//raw pointers are used to refer to the planning group for improved performance
	const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(select_group);
	if(new_positionX <= 0){
		link_end ="l_LEnd";
		group.setEndEffectorLink("l_LEnd");
	}
	else if(new_positionX > 0){
		link_end ="link_REnd";
		group.setEndEffectorLink("link_REnd");
	}
	else{};
	group.setPoseReferenceFrame("dummy");
	group.setPlannerId("RRTConnect");
	group.setNumPlanningAttempts(3);
	group.setPlanningTime(10.0);
	group.allowReplanning(true);
	//cai dat dung sai diem dich
	group.setGoalJointTolerance(0.01);
	group.setGoalPositionTolerance(0.01);
	group.setGoalOrientationTolerance(3.14);
	
	
	namespace rvt = rviz_visual_tools;
  	moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  	visual_tools.deleteAllMarkers();

  	// Remote control is an introspection tool that allows users to step through a high level script
  	// via buttons and keyboard shortcuts in RViz
  	visual_tools.loadRemoteControl();

  	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  	text_pose.translation().z() = 1.75;
 	 visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
 	 visual_tools.trigger();	


	//  CUSTOM PLANNING
	geometry_msgs::Pose target_pose1;
	//NOTE: THIS IS THE VALID POSE FROM RANDOM NODE

	   
	target_pose1.orientation.w = msg->orientation.w;
	target_pose1.orientation.x = msg->orientation.x;
	target_pose1.orientation.y = msg->orientation.y;
	target_pose1.orientation.z = msg->orientation.z;
	
	target_pose1.position.x =  msg->position.x;
	target_pose1.position.y =  msg->position.y;
	target_pose1.position.z =  msg->position.z;

	group.setStartStateToCurrentState();
   
	group.setPoseTarget(target_pose1,link_end);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit_msgs::MotionPlanRequest response;

	visual_tools.publishAxisLabeled(target_pose1, "pose1");
  	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  	visual_tools.trigger();
	group.plan(my_plan);

	group.move();

	
	ROS_INFO("pose updated");




	positionX = msg->position.x;
	positionY = msg->position.y;
	positionZ = msg->position.z;
	orientationX = msg->orientation.x;
	orientationY = msg->orientation.y;
	orientationZ = msg->orientation.z;
	orientationW = msg->orientation.w;
	
	}
	

}
void groupCallback(const std_msgs::String::ConstPtr& msg){
	select_group=msg->data.c_str();
}



int main(int argc, char **argv)
{

ros::init(argc, argv, "my_test_control");
ros::NodeHandle nh;
ROS_INFO("initilize !");


// define user callback queue
  ros::CallbackQueue string_queue;
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
      "/direction", // topic name
      1000, // queue length
      chatterCallback, // callback
      ros::VoidPtr(), // tracked object, we don't need one thus NULL
      &string_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber sub2 = nh.subscribe(ops);
  ros::Subscriber sub = nh.subscribe("select_group", 1000, groupCallback);

ros::AsyncSpinner spinner(1);
spinner.start();




ros::Rate r(10); // 10 hz
while (ros::ok())
  {
    ros::spinOnce();

    r.sleep();
  }

return 0;

}
