/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //planning group that we would like to control
	moveit::planning_interface::MoveGroupInterface l_group("l_arm");//group left arm
	moveit::planning_interface::MoveGroupInterface r_group("r_arm");// group right arm
	//we can add or remove collision objects in our virtual world scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	//raw pointers are used to refer to the planning group for improved performance
	const robot_state::JointModelGroup* joint_model_group = l_group.getCurrentState()->getJointModelGroup("l_arm");
	const robot_state::JointModelGroup* joint_model_r_group = r_group.getCurrentState()->getJointModelGroup("r_arm");

	//setup for left arm group
		l_group.setEndEffectorLink("l_LEnd");
		l_group.setPoseReferenceFrame("dummy");
		l_group.setPlannerId("RRTConnect");
		l_group.setNumPlanningAttempts(3);
		l_group.setPlanningTime(10.0);
		l_group.allowReplanning(true);
		//cai dat dung sai diem dich
		l_group.setGoalJointTolerance(0.01);
		l_group.setGoalPositionTolerance(0.01);
		l_group.setGoalOrientationTolerance(0.2);
	//setup for right arm group
		r_group.setEndEffectorLink("link_REnd");
		r_group.setPoseReferenceFrame("dummy");
		r_group.setPlannerId("RRTConnect");
		r_group.setNumPlanningAttempts(3);
		r_group.setPlanningTime(10.0);
		r_group.allowReplanning(true);
		//cai dat dung sai diem dich
		r_group.setGoalJointTolerance(0.01);
		r_group.setGoalPositionTolerance(0.01);
		r_group.setGoalOrientationTolerance(0.2);

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

	//group.setNamedTarget("random");
	//group.setRandomTarget();
	//group.move();  // WORKS FINE :)

	//  CUSTOM PLANNING
	

	//left arm goal   
	geometry_msgs::Pose target_pose1;
  	target_pose1.orientation.w = 0.109690891128  ;
  	target_pose1.orientation.x=-0.561145175475;
  	target_pose1.orientation.y=-0.284707810939;
 	target_pose1.orientation.z=-0.769431909161;
  	target_pose1.position.x = -0.110;
  	target_pose1.position.y = 0.201;
  	target_pose1.position.z = -0.374;
	// right arm goal
	geometry_msgs::Pose rtarget_pose1;
  	rtarget_pose1.orientation.w = 0.338193865006   ;
  	rtarget_pose1.orientation.x=-0.75435708242;
  	rtarget_pose1.orientation.y=-0.0451077395497;
 	rtarget_pose1.orientation.z=-0.560834729405;
  	rtarget_pose1.position.x = 0.110;
  	rtarget_pose1.position.y = 0.201;
  	rtarget_pose1.position.z = -0.374;
	

	//l arm planing
		l_group.setStartStateToCurrentState();
		l_group.setPoseTarget(target_pose1,"l_LEnd");
		//group.setPositionTarget(0.2,-0.2,0.2,"L_end");
		//moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		

	//r arm planning
		r_group.setStartStateToCurrentState();
		r_group.setPoseTarget(rtarget_pose1,"link_REnd");
		//group.setPositionTarget(0.2,-0.2,0.2,"L_end");
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		moveit_msgs::MotionPlanRequest response;
		
		r_group.plan(my_plan);
		l_group.plan(my_plan);

	 // We can also visualize the plan as a line with markers in RViz for l arm
  	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  	visual_tools.publishAxisLabeled(target_pose1, "pose1");
  	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  	visual_tools.trigger();

	// We can also visualize the plan as a line with markers in RViz for l arm
  	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  	visual_tools.publishAxisLabeled(rtarget_pose1, "poser1");
  	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_r_group);
  	visual_tools.trigger();	

	r_group.setGoalOrientationTolerance(3.14);
	l_group.setGoalOrientationTolerance(3.14);
	//group.plan(my_plan);
	//group.execute(my_plan);
	l_group.move();
	r_group.move();
	ROS_INFO("pose updated");

    	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  	// Cartesian Paths
	  // ^^^^^^^^^^^^^^^
	  // You can plan a Cartesian path directly by specifying a list of waypoints
	  // for the end-effector to go through. Note that we are starting
	  // from the new start state above.  The initial pose (start state) does not
	  // need to be added to the waypoint list but adding it can help with visualizations
	  std::vector<geometry_msgs::Pose> waypoints;
	  std::vector<geometry_msgs::Pose> rwaypoints;
	  geometry_msgs::Pose start_pose2 = target_pose1;
	  geometry_msgs::Pose rstart_pose2 = rtarget_pose1;
	  waypoints.push_back(start_pose2);
	  rwaypoints.push_back(rstart_pose2);
	  geometry_msgs::Pose rtarget_pose3 = rstart_pose2;
	  geometry_msgs::Pose target_pose3 = start_pose2;
	  for(int i=0; i<10;i++){
		  target_pose3.position.z += 0.02;
		  target_pose3.position.y += 0.02;
		  waypoints.push_back(target_pose3);  // right

	  }
	  for(int i=0; i<10;i++){
		  rtarget_pose3.position.z += 0.02;
		  rtarget_pose3.position.y += 0.02;
		 rwaypoints.push_back(rtarget_pose3);  // right

	  }

	  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
	  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
	  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
	  l_group.setMaxVelocityScalingFactor(0.1);

	  // We want the Cartesian path to be interpolated at a resolution of 1 cm
	  // which is why we will specify 0.01 as the max step in Cartesian
	  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
	  // Warning - disabling the jump threshold while operating real hardware can cause
	  // large unpredictable motions of redundant joints and could be a safety issue
	  moveit_msgs::RobotTrajectory trajectory;
	  l_group.setStartStateToCurrentState();
	  const double jump_threshold = 0.1;
	  const double eef_step = 0.02;
	  //double fraction = l_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	  //ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	  
	
	  // Visualize the plan in RViz
	  visual_tools.deleteAllMarkers();
	  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	  visual_tools.publishAxisLabeled(waypoints[waypoints.size()-1], "pt" + std::to_string(waypoints.size()-1), rvt::SMALL);
	  for (std::size_t i = 0; i < waypoints.size(); ++i){
	    //group.setPositionTarget(waypoints[i].position.x,waypoints[i].position.y,waypoints[i].position.z,"l_LEnd");
	    l_group.setGoalOrientationTolerance(3.14);
            l_group.setPoseTarget(waypoints[i],"l_LEnd");
            l_group.move();
	
	    r_group.setGoalOrientationTolerance(3.14);
            r_group.setPoseTarget(rwaypoints[i],"link_REnd");
            r_group.move();
	  }
          

	   
	  	  
  ros::shutdown();
  return 0;
}
