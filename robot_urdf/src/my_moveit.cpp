#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/SetBool.h>

int grab=0;

bool reach(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  
if(req.data){
	grab=1;
	resp.message = "grab_high";
	}
	else { 
	grab=2;
	resp.message = "grab_low";
	}
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  group.setStartStateToCurrentState();
  group.setGoalOrientationTolerance(0.01);
  group.setGoalPositionTolerance(0.01);
  
  
  while(ros::ok()){
	 if (grab==1){
	  group.setNamedTarget("pose1");
	  group.move();
	  grab=0;
  }
	  if (grab==2){
	  group.setNamedTarget("pose2");
	  group.move();
	  grab=0;
  }
  }
  return(0);
}
