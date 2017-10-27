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
#include "ros/ros.h"
#include "ros/package.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud.h"

#include "std_srvs/Empty.h"
#include <sstream>
// WiFi
#include "wifi_tools/WifiData.h"

//gridmap

#include "tug_2d_occupancy_grid_map/tug_2d_occupancy_grid_map.hpp"
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
    ros::Publisher gridmap_publisher;

    ros::ServiceServer out2file_srv_;
    tug_2d_occupancy_grid_map::Tug2dOccupancyGridMap occupancy_grid_map_;


};

WiFiMappingNode::WiFiMappingNode():
    nh_(),
    occupancy_grid_map_(nh_)
{

    nh_.param("wifiscan_topic",wifiscan_topic_,std::string("/wifi_tools/wifi_data"));
    nh_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    nh_.param("base_frame_id", base_frame_id_, std::string("/base_link"));

    tf_ = new tf::TransformListener();

    wifi_map_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/wifi_map", 10);

    wifiscan_sub_ = nh_.subscribe("/wifi_tools/wifi_data", 1,&WiFiMappingNode::wifiScanReceived,this);
    
    
    occupancy_grid_map_.init(2048, 2048, 1, -51.2, -51.2);

}
WiFiMappingNode::~WiFiMappingNode()
{
}

void WiFiMappingNode::wifiScanReceived(const wifi_tools::WifiData& wifi_msg)
{

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


    double z_x,z_y,z_s;
    ROS_INFO("Transformed message -> x: [%f], y: [%f] %d", transform.getOrigin().x(), transform.getOrigin().y(),wifi_msg.data );
    z_x = (double)transform.getOrigin().x();
    z_y = (double)transform.getOrigin().y();
    z_s = wifi_msg.data;
/*    n++;
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "odom";
    cloud.points.resize(n);
    cloud.points[0].x = z_x;
    cloud.points[0].y = z_y;
    cloud.points[0].z = z_ss;
    ros::spinOnce();
    wifi_map_pub_.publish(cloud);

    gridmap_publisher =nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    // Create grid map.
    GridMap map({"wifi"});
    map.setFrameId("map");
    map.setGeometry(Length(1.2, 2.0), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(),
            map.getSize()(0), map.getSize()(1));

    // Work with grid map in a loop.
    ros::Rate rate(30.0);
    while (nh_.ok()) {

        // Add data to grid map.
        ros::Time time = ros::Time::now();
        for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
            Position position;
            map.getPosition(*it, position);
            map.at("wifi", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
        }

        // Publish grid map.
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.info.header.stamp.toSec());

        // Wait for next cycle.
        rate.sleep();
    
*/
    int map_x;
    int map_y;
    if(occupancy_grid_map_.worldToMap(z_x, z_y, map_x, map_y) )
    {ROS_INFO("map -> x: [%d], y: [%d]", map_y,map_x );
        occupancy_grid_map_.updateCell(map_x,map_y, z_s);}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "WiFi_mapping_node");
    ros::NodeHandle nh;
    WiFiMappingNode wifi_mapping;

    ros::spin();
    return (0);
}
