#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <string.h>
#include <iostream>
#include <laser_assembler/AssembleScans.h>

#define PI 3.1415

using namespace std;
using namespace laser_assembler;

class BackAndForthMovement{
	public:
		float pos_limit_; // position limit (rad)
		float w_limit_; // angular velocity limit (rad/s)
		
		sensor_msgs::JointState joint_command_; // joint to send
		sensor_msgs::JointState joint_state_;   // joint to receive
		
		ros::NodeHandle nh;
		ros::Publisher joint_pub_;
  		ros::Publisher laser_3d_scan_start_;
		ros::Subscriber joint_sub_;
		int state_;
  		int last_state_;
		double desired_freq_;
		string name_;
		bool joint_received_;
		ros::Rate *loop_rate_;
		
		double offset_;
		double uncertainty_;

  		bool make_3d_scan_;
  		ros::Subscriber laser_2d_sub_;
  		ros::Subscriber sweep_button_trigger_;

  		ros::Publisher pub_;
  		ros::ServiceClient client_;

  		float pos_limit_horizontal_left_;
  		float pos_limit_horizontal_right_;

        bool sweep_triggered_;

		
		BackAndForthMovement()
		{
			//ROS_INFO("back and forth movement started");
			
			pos_limit_ = PI / 2;// - 0.02; //-0.02 is used to avoid sign changing on +/- PI/2
			pos_limit_horizontal_left_ = 0.0;
			pos_limit_horizontal_right_ = PI;

			w_limit_ = 0.1;
			state_ = 1;
			desired_freq_ = 100.0;
			name_ = "H42";
			joint_received_ = false;
			loop_rate_ = new ros::Rate(desired_freq_);
			
			offset_ = 0;
			uncertainty_ = 0.03;

			make_3d_scan_ = false;

			sweep_triggered_ = false;

			joint_state_ = sensor_msgs::JointState();

			//joint_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_commands",1);
            //joint_pub_ = nh.advertise<std_msgs::Float64>("/la_roll_controller/command",1);
			joint_pub_ = nh.advertise<std_msgs::Float64>("/p3at/la_gazebo_joint_controller",1);
            //joint_pub_ = nh.advertise<std_msgs::Float64>("/p3at/la_roll_joint_position_controller/command",1);
            //joint_sub_ = nh.subscribe<sensor_msgs::JointState>("/la_joint_state", 1, &BackAndForthMovement::jointStateCallback, this);
			joint_sub_ = nh.subscribe<sensor_msgs::JointState>("/p3at/joint_states", 1, &BackAndForthMovement::jointStateCallback, this);


			laser_3d_scan_start_ = nh.advertise<std_msgs::Bool>("/laser/start3dscan", 1);

			sweep_button_trigger_ = nh.subscribe<std_msgs::String>("/trigger_laser_sweep", 1, &BackAndForthMovement::triggerSweepCallback, this);
			/*//laser_2d_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/laser/scan", 2, laserCloudHandler);
				// Create a publisher for the clouds that we assemble
				pub_ = nh.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

				// Create the service client for calling the assembler
				client_ = nh.serviceClient<AssembleScans>("assemble_scans");
*/
			//ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud/hokuyo", 2, laserCloudHandler);

            //center the laser
            std_msgs::Float64 value;
            value.data = offset_;
            joint_pub_.publish(value);

			//controlLoop();
            while(ros::ok())
			{
               controlLoop();
            }
		}

		
  void triggerSweepCallback(const std_msgs::String trigger)
  {
	  //ROS_INFO("Laser sweep triggered.");
      sweep_triggered_  = true;
  }

  void jointStateCallback(const sensor_msgs::JointState joint_state)
  {
	  joint_received_ = true;

	  // copy joint state msg
	  joint_state_.position.clear();
	  joint_state_.name.clear();
	  joint_state_.name.push_back(joint_state.name[0]);
	  joint_state_.position.push_back((joint_state.position[0] - offset_)); //Michael_Comment: added % PI
  }


	void controlLoop()
	{
		ROS_INFO("inside controlLoop");
		//ros::Rate loop_rate_(30);
		//loop_rate_.reset();
		while(ros::ok())
		{
            //wait for input key pressed
            //std::string inputString;
            //std::cout << "Press 'Enter' to start laser sweep:\n";
            //std::getline(std::cin, inputString);

			// only move the joint when a joint_state msg is received
			if(joint_received_)
			{
				if(state_ == 1)
					initialize();
				else if(state_ == 2)
					turnLeft();
				else if(state_ == 3)
					turnRight();
			}
			joint_received_ = false;

			ros::spinOnce();
			loop_rate_->sleep();
		}
		ros::shutdown();
	}

  /*!
   *  /brief Initializes joint position to 0
   */
  void initialize()
  {
	  ROS_INFO("Initializing joint");

	  // if not centered
	  if(std::fabs(joint_state_.position[0]) >= uncertainty_)
	  {
		  //ROS_INFO("not centered");
		  //ROS_INFO("pos=%f | limit=%f",joint_state_.position[0], uncertainty_);
		  std_msgs::Float64 center_joint;
		  center_joint.data = 0.0; // + offset_ + uncertainty_;
		  // @todo: set velocity depending the direction it has to move
		  //ROS_INFO("sending target_position: %f",center_joint.data);
		  joint_pub_.publish(center_joint);
	  }
		  // if centered
	  else
	  {
		  //ROS_INFO("centered");
		  state_ = 2; // turn left
		  pressEnterKey();
		  pos_limit_ = pos_limit_horizontal_right_;// - offset_ - uncertainty_;

		  //ros::Duration duration(1./5.);
		  //duration.sleep();
	  }
	  //ROS_INFO("end: Initializing joint");
  }

  void turnLeft()
  {
	  ROS_INFO("Turning left");

	  // limit not reached
	  if(joint_state_.position[0] < pos_limit_ - uncertainty_)
	  {
		  //ROS_INFO("limit not reached");
		  //ROS_INFO("pos=%f | limit=%f",joint_state_.position[0], pos_limit_);
		  std_msgs::Float64 left_joint;
		  left_joint.data = pos_limit_;// + offset_;// + uncertainty_;
		  joint_pub_.publish(left_joint);
		  //ROS_INFO("target-pos=%f",left_joint.data);
	  }
		  // limit reached
	  else
	  {
		  //ROS_INFO("limit reached");
		  state_ = 3; // turn right
		  std_msgs::Bool start;
		  start.data = false;
		  laser_3d_scan_start_.publish(start);
		  pressEnterKey();
		  start.data = true;
		  laser_3d_scan_start_.publish(start);
		  pos_limit_ = pos_limit_horizontal_left_;// + offset_;// - uncertainty_;

		  ros::Duration duration(1./8.);
		  duration.sleep();
	  }
	  //ROS_INFO("end: Turning left");
  }

  void turnRight()
  {
	  ROS_INFO("Turning right");

	  // limit not reached
	  if(joint_state_.position[0] > -pos_limit_ + uncertainty_)
	  {
		  //ROS_INFO("limit not reached");
		  //ROS_INFO("pos=%f | limit=%f",joint_state_.position[0], pos_limit_);
		  std_msgs::Float64 right_joint;
		  right_joint.data = -pos_limit_;// + offset_ - uncertainty_;
		  joint_pub_.publish(right_joint);
		  //ROS_INFO("target-pos=%f",right_joint.data);
	  }
		  // limit reached
	  else
	  {
		  //ROS_INFO("limit reached");
		  state_ = 2; // turn right
		  std_msgs::Bool start;
		  start.data = false;
		  laser_3d_scan_start_.publish(start);
		  pressEnterKey();
		  start.data = true;
		  laser_3d_scan_start_.publish(start);
		  pos_limit_ = pos_limit_horizontal_right_;// - offset_ - uncertainty_;

		  ros::Duration duration(1./8.);
		  duration.sleep();
	  }
	  //ROS_INFO("end: Turning right");
  }

  void pressEnterKey()
  {
	  //wait for input key pressed
	  /*std::string inputString;
	  std::cout << "Press 'Enter' to start laser sweep:\n";
	  std::getline(std::cin, inputString);*/
	  ros::Duration duration(1./3.);
	  sweep_triggered_  = false;
	  while(ros::ok() && (sweep_triggered_ == false))
	  {
		  //ROS_INFO("sleep");
		  duration.sleep();
		  ros::spinOnce();
	  }
	  sweep_triggered_ = false;
  }
}; // end class


int main(int argc, char** argv)
{
	ros::init(argc, argv, "back_and_forth_movement");
	BackAndForthMovement backAndForthMovement;
	return 0;
}

