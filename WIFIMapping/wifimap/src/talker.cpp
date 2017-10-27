#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "wifi_tools/WifiData.h"


#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n1;

  ros::Publisher chatter_pub = n1.advertise<wifi_tools::WifiData>("/wifi_tools/wifi_data", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    //std_msgs::String msg;
    wifi_tools::WifiData msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/odom";

    msg.data =150;
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}

