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
#include "position_msgs/ObjectPositions.h"

ros::Publisher select_ob;
ros::Publisher gripper;
//
float positionX = -0.155026881889;
float positionY = -0.00380711320647;
float positionZ = -0.580344179492;
position_msgs::ObjectPositions objectpositionresults;
float orientationX =  0.712362483218;
float orientationY = 0.701773892725;
float orientationZ = 0.00167994660707;
float orientationW = 0.00709040002731 ;
std::string select_group="dual_arm";
std::string select_object="";
std::string old_object="a";
std::string link_end ="l_LEnd";
void chatterCallback(const position_msgs::ObjectPositions::ConstPtr& msg)
{
  if(select_object!=""){
	//int i=0;
	for(int i = 0;i<msg->object_positions.size();i++ ){
		  if(msg->object_positions[i].Class==select_object && old_object!=select_object){
			float new_positionX = float(msg->object_positions[i].x);
			float new_positionY = float(msg->object_positions[i].y);
			float new_positionZ = float(msg->object_positions[i].z);

			ROS_INFO("saved_x: [%.9f], new:[%.9f]", positionX, new_positionX);
			ROS_INFO("saved_y: [%.9f], new [%.9f]", positionY, new_positionY);
			ROS_INFO("saved_z: [%.9f], new [%.9f]", positionZ, new_positionZ);
		 	old_object=select_object;
			//std_msgs::String ms;
			//ms.data="";
			//select_ob.publish(ms);
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
				ROS_INFO("l_arm");
			}
			else if(new_positionX > 0){
				link_end ="link_REnd";
				group.setEndEffectorLink("link_REnd");
				ROS_INFO("r_arm");
			}
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

			   
			target_pose1.orientation.w = 0;
			target_pose1.orientation.x = 0;
			target_pose1.orientation.y = 0;
			target_pose1.orientation.z = 0;
			
			target_pose1.position.x =  float(msg->object_positions[i].x)/1000;
			target_pose1.position.y =  float(msg->object_positions[i].y)/1000;
			target_pose1.position.z =  float(msg->object_positions[i].z)/1000;

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
			positionX = float(msg->object_positions[i].x);
			positionY = float(msg->object_positions[i].y);
			positionZ = float(msg->object_positions[i].z);
			if(link_end=="l_LEnd"){
				std_msgs::String gr;
				gr.data="gL";
				gripper.publish(gr);
			}
			if(link_end=="link_REnd"){
				std_msgs::String gr;
				gr.data="gR";
				gripper.publish(gr);
			}
			
			}
			break;
		   }
			//i++;
	}
   }

}
void select_objectCallback(const std_msgs::String::ConstPtr& msg){
	select_object=msg->data.c_str();
}



int main(int argc, char **argv)
{

ros::init(argc, argv, "dual_arm_control");
ros::NodeHandle nh;
ROS_INFO("initilize !");
ros::CallbackQueue string_queue;
ros::SubscribeOptions position =
    ros::SubscribeOptions::create<position_msgs::ObjectPositions>(
      "/objects_position/message", // topic name
      1000, // queue length
      chatterCallback, // callback
      ros::VoidPtr(), // tracked object, we don't need one thus NULL
      &string_queue // pointer to callback queue object
    );

// define user callback queue
  
  // create options for subscriber and pass pointer to our custom queue
  // subscribe
  ros::Subscriber sub2 = nh.subscribe(position);
  ros::Subscriber sub = nh.subscribe("/select_object", 1000, select_objectCallback);
 // select_ob = nh.advertise<std_msgs::String>("/select_object", 10);
  gripper= nh.advertise<std_msgs::String>("/gripper", 10);

ros::AsyncSpinner spinner(1, &string_queue);
spinner.start();




ros::Rate r(10); // 10 hz
while (ros::ok())
  {
    ros::spinOnce();

    r.sleep();
  }

return 0;

}
