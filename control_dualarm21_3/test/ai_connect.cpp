#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
using namespace std;

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

// vui ve, hello = "0" => vay tay
// buon, iloveyou = "1" => dua tay ra truoc (trai tim)
// cam on = "3" => dua 2 tay len tren (trai tim)
// binh thuong = "2" => khong lam gi


std::string select_group="dual_arm";
std::string link_end ="l_LEnd";
std::string old_emotion;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

	moveit::planning_interface::MoveGroupInterface group(select_group);
	

	//raw pointers are used to refer to the planning group for improved performance
	const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(select_group);

	
	group.setPoseReferenceFrame("dummy");
	group.setPlannerId("RRTConnect");
	group.setNumPlanningAttempts(3);
	group.setPlanningTime(30.0);
	group.allowReplanning(true);
	//cai dat dung sai diem dich
	group.setGoalJointTolerance(0.05);
	group.setGoalPositionTolerance(0.02);
	group.setGoalOrientationTolerance(0.01);
	
	std::string number=msg->data.c_str();

	if(number=="1"&&number!=old_emotion){

	/*	group.setStartStateToCurrentState();
   
	group.setPoseTarget(target_pose1,link_end);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit_msgs::MotionPlanRequest response;

	visual_tools.publishAxisLabeled(target_pose1, "pose1");
  	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  	visual_tools.trigger();
	group.plan(my_plan);

	group.move();*/
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[0] = -1.581719699821226;  // radians
	joint_group_positions[1] = 0.8558882979544471;  // radians
	joint_group_positions[2] = -0.5311135886043487;  // radians
	joint_group_positions[3] = -1.0918415603302891;  // radians
	joint_group_positions[4] = -1.5699398011889776;  // radians
	joint_group_positions[5] = 1.5699535045717459;  // radians
	joint_group_positions[6] = -0.7260424399441093;  // radians
	joint_group_positions[7] =  0.5843105956946987;  // radians
	joint_group_positions[8] = 0.7377079607459863;  // radians
	joint_group_positions[9] = -1.5699659519333626;  // radians
	group.setJointValueTarget(joint_group_positions);
	moveit_msgs::MotionPlanRequest response;
	group.plan(my_plan);
	group.move();
	
	old_emotion=number;
	}
	else if(number=="3"&&number!=old_emotion){

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[0] = 0;  // radians
	joint_group_positions[1] = 2.531984256010908;  // radians
	joint_group_positions[2] = 0.601962442078165;  // radians
	joint_group_positions[3] = 1.2394827588997392;  // radians
	joint_group_positions[4] = 0;  // radians
	joint_group_positions[5] = 0;  // radians
	joint_group_positions[6] = -2.5319728896854725;  // radians
	joint_group_positions[7] =  -0.6137802269368029;  // radians
	joint_group_positions[8] = -1.215816863430567;  // radians
	joint_group_positions[9] = 0;  // radians
	group.setJointValueTarget(joint_group_positions);
	moveit_msgs::MotionPlanRequest response;
	group.plan(my_plan);
	group.move();
	old_emotion=number;
	}
	else if(number=="0"&&number!=old_emotion){

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[0] = 0;  // radians
	joint_group_positions[1] = 0;  // radians
	joint_group_positions[2] = 0;  // radians
	joint_group_positions[3] = 0;  // radians
	joint_group_positions[4] = 0;  // radians
	joint_group_positions[5] = 0.05501679940354334;  // radians
	joint_group_positions[6] = -2.1898072784741274;  // radians
	joint_group_positions[7] =  -1.1390367877297027;  // radians
	joint_group_positions[8] = -0.41306919087007965;  // radians
	joint_group_positions[9] = 1.5699069820937235;  // radians
	group.setJointValueTarget(joint_group_positions);
	moveit_msgs::MotionPlanRequest response;
	group.plan(my_plan);
	group.move();
	old_emotion=number;
	}
	else{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[0] = 0;  // radians
	joint_group_positions[1] = 0;  // radians
	joint_group_positions[2] = 0;  // radians
	joint_group_positions[3] = 0;  // radians
	joint_group_positions[4] = 0;  // radians
	joint_group_positions[5] = 0;  // radians
	joint_group_positions[6] = 0;  // radians
	joint_group_positions[7] = 0;  // radians
	joint_group_positions[8] = 0;  // radians
	joint_group_positions[9] = 0;  // radians
	group.setJointValueTarget(joint_group_positions);
	moveit_msgs::MotionPlanRequest response;
	group.plan(my_plan);
	group.move();
	old_emotion=number;
	}

}



int main(int argc, char **argv)
{

ros::init(argc, argv, "ai_connect");
ros::NodeHandle nh;
ROS_INFO("initilize !");
ros::CallbackQueue string_queue;


// define user callback queue
  
  // create options for subscriber and pass pointer to our custom queue
   
  // subscribe
  ros::Subscriber sub = nh.subscribe("/emotion_publisher",1000,chatterCallback);
  //ros::Subscriber sub = nh.subscribe("gesture_publisher", 1000, chatterCallback); 
 
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
