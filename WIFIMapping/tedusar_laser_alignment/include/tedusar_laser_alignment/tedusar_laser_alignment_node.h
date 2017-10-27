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

#ifndef la_joint_state_publisher_h___
#define la_joint_state_publisher_h___

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

namespace tedusar_laser_alignment
{

class JointStatePublisher
{
protected:
    ros::NodeHandle nh_;

    ros::Subscriber servo_1_state_sub_;
    ros::Subscriber servo_2_state_sub_;

    ros::Publisher joint_state_pub_;

    ros::Timer publish_joint_state_timer_;

    sensor_msgs::JointState la_joint_states_;

public:
    JointStatePublisher(ros::NodeHandle nh);

    virtual ~JointStatePublisher();

    void init();


protected:
    void servo1StateMsgCB(const dynamixel_msgs::JointState::ConstPtr state);

    void servo2StateMsgCB(const dynamixel_msgs::JointState::ConstPtr state);

    void publishJointStateTimerCB(const ros::TimerEvent& e);
};

} // end namespace tedusar_laser_alignment

#endif // la_joint_state_publisher_h___
