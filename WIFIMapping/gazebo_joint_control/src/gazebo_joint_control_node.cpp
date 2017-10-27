#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <iostream>
#include <laser_assembler/AssembleScans.h>

#define PI 3.1415

using namespace std;
using namespace laser_assembler;

class JointMover
{
 public:
  float pos_limit_; // position limit (rad)
  float w_limit_; // angular velocity limit (rad/s)

  sensor_msgs::JointState joint_command_; // joint to send
  sensor_msgs::JointState joint_state_;   // joint to receive

  ros::NodeHandle nh;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber joint_sub_actual_;
  int state_;
  double desired_freq_;
  string name_;
  bool target_received_;
  ros::Rate *loop_rate_;

  double offset_;
  double uncertainty_;

  bool make_3d_scan_;
  ros::Subscriber sub_;

  ros::Publisher pub_;

  float current_position_;
  float target_position_;
  bool cancel_movement_;
  float end_position_uncertainty_;

  JointMover()
  {
    //ROS_INFO("JointMover started");

    pos_limit_ = PI / 2;
    w_limit_ = 0.1;
    state_ = 1;
    desired_freq_ = 100.0;
    name_ = "H42";
    target_received_ = false;
    loop_rate_ = new ros::Rate(desired_freq_);

    offset_ = 0;
    uncertainty_ = 0.0;//0.02;

    current_position_ = 0.0;
    cancel_movement_ = false;
    //end_position_uncertainty_ = 0.05;

    joint_state_ = sensor_msgs::JointState();

    joint_pub_ = nh.advertise<std_msgs::Float64>("/p3at/la_roll_joint_position_controller/command", 1);
    joint_sub_ = nh.subscribe<std_msgs::Float64>("/p3at/la_gazebo_joint_controller", 1, &JointMover::jointNewTargetPosition, this);

    joint_sub_actual_ = nh.subscribe<sensor_msgs::JointState>("/p3at/joint_states", 1, &JointMover::jointStateCallback, this);

    //center the laser
    std_msgs::Float64 value;
    value.data = 0.0;
    joint_pub_.publish(value);

    //controlLoop();
    while (ros::ok())
    {
      if (target_received_ == true)
        move();

      ros::spinOnce();
      loop_rate_->sleep();
    }
  }

  void move()
  {
    //ROS_INFO("entering: move");
    current_position_ = joint_state_.position[0];
    cancel_movement_ = false;
    int counter = 0;
    ros::Duration duration(1. / 100.);//24.);
    std_msgs::Float64 value;
    value.data = current_position_;
    float step_size = 0.01;//0.05;
    float step = step_size;
    float stop;

    if (target_position_ > 0.0 && current_position_ < 0.0)
    {
      step = step_size;
      stop = target_position_ + step;
    }
    else if (target_position_ < 0.0 && current_position_ > 0.0)
    {
      step = -step_size;
    }
    else if (target_position_ > current_position_)
    {
      step = step_size;
    }
    else if (target_position_ < current_position_)
    {
      step = -step_size;
    }
    else
      return;

    stop = target_position_ + step;

    while (ros::ok() && cancel_movement_ == false)
    {
      value.data += step;
      joint_pub_.publish(value);
      //ROS_ERROR("value = %f",value.data);
      current_position_ = value.data;
      duration.sleep();

      if (step>0 && current_position_ >= stop)
      {
        //ROS_ERROR("STOP1 reached");
        break;
      }
      else if (step<0 && current_position_ <= stop)
      {
        //ROS_ERROR("STOP2 reached");
        break;
      }
    }
    cancel_movement_ = true;
    target_received_ = false;
    //ROS_INFO("exiting: move");
  }


  void jointNewTargetPosition(const std_msgs::Float64 target_position)
  {
    //ROS_INFO("got new target: %f", target_position.data);
    current_position_ = joint_state_.position[0];
    if(std::abs(current_position_ - target_position.data) > 0.1)
    {
      target_received_ = true;
      cancel_movement_ = true;
      target_position_ = target_position.data;
    }
    else
    {
      std_msgs::Float64 value;
      value.data = target_position.data;
      joint_pub_.publish(value);
      //ROS_INFO("new target differs just in: %f so ignore it", std::abs(current_position_ - target_position.data));
    }
  }

  void jointStateCallback(const sensor_msgs::JointState joint_state)
  {
    //joint_received_ = true;

    // copy joint state msg
    //current_position_ = joint_state.position[0];
    joint_state_.position.clear();
    joint_state_.name.clear();
    joint_state_.name.push_back(joint_state.name[0]);
    joint_state_.position.push_back((joint_state.position[0] - offset_)); //Michael_Comment: added % PI
  }

}; // end class


int main(int argc, char **argv)
{
  ros::init(argc, argv, "JointMover");

  JointMover mover;

  ros::spin();

  return 0;
}
