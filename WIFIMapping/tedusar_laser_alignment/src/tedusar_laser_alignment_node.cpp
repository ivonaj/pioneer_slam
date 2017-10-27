/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Maurer,
 *                      Institute for Software Technology,
 *                      Graz University of Technology
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
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
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
 *
 *********************************************************************/

#include <tedusar_laser_alignment/tedusar_laser_alignment_node.h>

namespace tedusar_laser_alignment
{

JointStatePublisher::JointStatePublisher(ros::NodeHandle nh) :
    nh_(nh)
{

}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::init()
{
    ros::NodeHandle private_nh("~");

    std::string roll_joint_name, pitch_joint_name;
    private_nh.param<std::string>("roll_joint_name", roll_joint_name, std::string("la_roll_joint"));
    private_nh.param<std::string>("pitch_joint_name", pitch_joint_name, std::string("la_pitch_joint"));

    double frequency;
    private_nh.param<double>("frequency", frequency, 30.0);

    la_joint_states_.name.resize(2);
    la_joint_states_.position.resize(2);
    la_joint_states_.name[0] = roll_joint_name;
    la_joint_states_.position[0] = 0.0;
    la_joint_states_.name[1] = pitch_joint_name;
    la_joint_states_.position[1] = 0.0;

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("la_joint_state", 1);

    servo_1_state_sub_ = nh_.subscribe("servo_roll_state", 1, &JointStatePublisher::servo1StateMsgCB, this);
    servo_2_state_sub_ = nh_.subscribe("servo_pitch_state", 1, &JointStatePublisher::servo2StateMsgCB, this);

    publish_joint_state_timer_ = nh_.createTimer(ros::Duration(1.0/frequency), &JointStatePublisher::publishJointStateTimerCB, this, false );
}

void JointStatePublisher::servo1StateMsgCB(const dynamixel_msgs::JointState::ConstPtr state)
{
    la_joint_states_.position[0] = state->current_pos;
    la_joint_states_.header.stamp = state->header.stamp;
}

void JointStatePublisher::servo2StateMsgCB(const dynamixel_msgs::JointState::ConstPtr state)
{
    la_joint_states_.position[1] = state->current_pos;
}

void JointStatePublisher::publishJointStateTimerCB(const ros::TimerEvent& e)
{
//    la_joint_states_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(la_joint_states_);
}

} // end namespace tedusar_laser_alignment

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc,argv,"tedusar_laser_alignment_node");

        ros::NodeHandle nh;

        tedusar_laser_alignment::JointStatePublisher joint_state_publisher(nh);

        joint_state_publisher.init();

        ros::spin();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }


    return 0;
}
