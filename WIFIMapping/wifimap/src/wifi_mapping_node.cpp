/*
 * Copyright (c) 2013, Seigo ITO
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Seigo ITO nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// ROS
 #include <hector_gazebo_plugins/gazebo_ros_wireless_receiver.h>
#include "ros/ros.h"
#include "ros/package.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "std_srvs/Empty.h"

// WiFi
#include "wifi_tools/WifiData.h"
#include "wifi_tools/Out2File.h"



class WiFiMappingNode
{
public:
  WiFiMappingNode();
  ~WiFiMappingNode();

  void wifiScanReceived(const wifi_tools::WifiData& wifi_msg);
  bool outputFile(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
  //message_filters::Subscriber<wifi_tools::WifiData>* wifiscan_sub_;
  ros::Subscriber wifiscan_sub_;
  tf::MessageFilter<wifi_tools::WifiData>* wifi_filter_;

  ros::NodeHandle nh_;
  tf::TransformListener* tf_;

  // frame_id for odom
  std::string odom_frame_id_;
  // frame_id for base
  std::string base_frame_id_;

  // topic of wifi_scan
  std::string wifiscan_topic_;

  ros::Publisher wifi_map_pub_;

  ros::ServiceServer out2file_srv_;
};



WiFiMappingNode::WiFiMappingNode()
{
  nh_.param("wifiscan_topic",wifiscan_topic_,std::string("/wifi_tools/wifi_data"));
  nh_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
  nh_.param("base_frame_id", base_frame_id_, std::string("/base_link"));

  tf_ = new tf::TransformListener();
/*
  // wifiscan subscriber
  wifiscan_sub_ = new message_filters::Subscriber<wifi_tools::WifiData>(nh_, wifiscan_topic_, 100); //scanned wifi info!!
  
  // Mesage integration
  wifi_filter_     =  new tf::MessageFilter<wifi_tools::WifiData>(*wifiscan_sub_, *tf_, odom_frame_id_, 100);
  wifi_filter_->registerCallback(boost::bind(&WiFiMappingNode::wifiScanReceived,this, _1));
  */
  // wifi map publisher
  wifi_map_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/wifi_map", 10);


  wifiscan_sub_ = nh_.subscribe("/wifi_tools/wifi_data", 1,&WiFiMappingNode::wifiScanReceived,this);

}



WiFiMappingNode::~WiFiMappingNode()
{
}


void WiFiMappingNode::wifiScanReceived(const wifi_tools::WifiData& wifi_msg)
{
  // *** tf pose associated with this massage ***
  //tf::TransformListener listener;
  //tf::Stamped<tf::Pose> odom_pose;
  //tf::StampedTransform transform;
  //tf::Stamped<tf::Pose> base_pose (tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(0), base_frame_id_);
  //tf::Stamped<tf::Pose> odom_pose (tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(0), odom_frame_id_);
  /*try
  { ros::Time now = ros::Time::now();
    listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
    //this->tf_->transformPose(odom_frame_id_, base_pose, odom_pose);
  }
  catch(tf::TransformException e)
  {
          ROS_WARN("can't get transform. (%s)", e.what());
  }*/

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    bool found = false;
    ros::Time now ;

    try
    {
      ros::Time now = ros::Time(0); // get latest available transform
      found = tf_listener.waitForTransform("/odom", "/base_link", now, ros::Duration(5.0), ros::Duration(0.01));

      if (found)
      {
        tf_listener.lookupTransform("/odom", "/base_link", now, transform);

      }
    }
    catch (tf::TransformException & ex)
    {
        ROS_INFO_STREAM("No transform possible: " << ex.what());
    }


  double z_x,z_y,z_ss;
    ROS_INFO("Transformed message -> x: [%f], y: [%f] %d", transform.getOrigin().x(), transform.getOrigin().y(),wifi_msg.data );
    //z_x = (double)odom_pose.getOrigin().x();
    //z_y = (double)odom_pose.getOrigin().y();
    //z_ss = (double)wifi_msg->data[i].ss; // signal strength
    z_x = (double)transform.getOrigin().x();
    z_y = (double)transform.getOrigin().y();
    z_ss = wifi_msg.data;

    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "odom";
    cloud.points.resize(1);
    cloud.points[0].x = z_x;
    cloud.points[0].y = z_y;
    cloud.points[0].z = z_ss;
    ros::spinOnce();
    wifi_map_pub_.publish(cloud);
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "WiFi_mapping_node");
  ros::NodeHandle nh;
  WiFiMappingNode wifi_mapping;

    ros::spin();

  return (0);
}
