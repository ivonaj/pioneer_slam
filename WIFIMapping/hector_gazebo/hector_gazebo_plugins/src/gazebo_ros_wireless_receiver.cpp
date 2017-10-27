//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_gazebo_plugins/gazebo_ros_wireless_receiver.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/WirelessReceiver.hh>
#include <gazebo/sensors/WirelessTransmitter.hh>
#include <std_msgs/Int16.h>
#include <limits>

#include "gazebo/physics/Base.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/WorldState.hh"
#include "gazebo/gazebo.hh"

#include <gazebo/sensors/SensorManager.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "wifi_tools/WifiData.h"
#include <sstream>

using namespace std;

namespace gazebo {

GazeboRosWirelessReceiver::GazeboRosWirelessReceiver()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosWirelessReceiver::~GazeboRosWirelessReceiver()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();

  updateTimerNew.Disconnect(updateConnectionNew);


  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosWirelessReceiver::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  ROS_ERROR("-- GazeboRosWirelessReceiver::Load");
  // Get then name of the parent sensor
  sensor_ = boost::dynamic_pointer_cast<sensors::Sensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosWirelessReceiver requires a Sensor as its parent");
    return;
  }

  //Testing: I keep the current gazebo wireless plugin which uses just the distance and also implement the one using the RSSI
  ROS_ERROR("-- GazeboRosWirelessReceiver::Load: getting the WirelessReceiver");
  // Get the parent sensor.
  this->parentSensor =
      boost::dynamic_pointer_cast<sensors::WirelessReceiver>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "WirelessReceiverPlugin requires a WirelessReceiver.\n";
    return;
  }
  ROS_ERROR("-- GazeboRosWirelessReceiver::Load: got the WirelessReceiver");


  /*ROS_ERROR("-- GazeboRosWirelessReceiver::Load: getting the WirelessTransmitter");
  // Get the parent sensor.
  this->transmitterSensor =
      boost::dynamic_pointer_cast<sensors::WirelessTransmitter>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->transmitterSensor)
  {
    gzerr << "WirelessTransmitterPlugin requires a WirelessTransmitter.\n";
    return;
  }
  ROS_ERROR("-- GazeboRosWirelessReceiver::Load: got the WirelessTransmitter");*/



  //Reset();



  /* this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&GazeboRosWirelessReceiver::OnUpdate, this));*/


  //Testing: End

  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);
  std::string parentName = sensor_->GetParentName();
  parentEntity = world->GetEntity(parentName);

  // default parameters
  namespace_.clear();
  topic_ = "wireless_receiver";
  frame_id_ = "/wireless_receiver";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();



  rss_sensor_model_.Load(_sdf, "rss");
  AoA_sensor_model_.Load(_sdf, "AoA");


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  physics::Model_V models = this->world->GetModels();
  transmitter_count_ = 0;
  for(physics::Model_V::iterator it = models.begin(); it != models.end(); it++)
  {
    std::string name = (*it)->GetName();

    //ROS_INFO("-- GazeboRosWirelessReceiver::Load:: found transmitter name=%s",name.c_str());
    if( name.compare(0, 15, "wireless_router") == 0 )
    {
      transmitter_count_++;
    }
  }
  if(transmitter_count_ > 0)
  {
    transmitter_pub_ = node_handle_->advertise<sensor_msgs::PointCloud>(topic_+"/transmitter", 1);
    rss_pub_ = node_handle_->advertise<sensor_msgs::PointCloud>(topic_+"/rss", 1);
    AoA_pub_ = node_handle_->advertise<sensor_msgs::PointCloud>(topic_+"/AoA", 1);
    receiver_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(topic_ + "/receiver", 1);
    wifi_pub = node_handle_->advertise<wifi_tools::WifiData>("/wifi_tools/wifi_data", 1); // topic for publishing wi-fi data
  }


  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &rss_sensor_model_, _1, _2));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &AoA_sensor_model_, _1, _2));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(10.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosWirelessReceiver::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);


  // Connect to the sensor update event.
  // connect Update function
  updateTimerNew.setUpdateRate(10.0);
  updateTimerNew.Load(world, _sdf);
  updateConnectionNew = updateTimerNew.Connect(boost::bind(&GazeboRosWirelessReceiver::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Store the pointer to the wireless transmitter
  //TODO: specify and read out the name for the transmitter element. An hardcoded string is no good.
  this->transmitterSensor = boost::static_pointer_cast<sensors::WirelessTransmitter>(
      sensors::SensorManager::Instance()->GetSensor("wirelessTransmitter"));
}

void GazeboRosWirelessReceiver::Reset()
{
  updateTimer.Reset();
  rss_sensor_model_.reset();
  AoA_sensor_model_.reset();

  //Edit:
  updateTimerNew.Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosWirelessReceiver::Update()
{
  ROS_ERROR("-- GazeboRosWirelessReceiver::Update()");
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  if(transmitter_count_  == 0)
  {
    ROS_INFO("-- GazeboRosWirelessReceiver::Update()::  transmitter_count_  == 0");
    return;
  }

  //ROS_INFO("-- GazeboRosWirelessReceiver::Update():: transmitter_count_ == %d",transmitter_count_);
  
  sensor_msgs::PointCloud transmitters_;//positon of transmitter
  sensor_msgs::PointCloud rss_; //rss value of transmitter, here is the distance 
  sensor_msgs::PointCloud AoA_;//angle of arrival 
  geometry_msgs::PoseStamped receiver_pose_;

  //receiver's pose in the world
  math::Pose referencePose = 
      sensor_->GetPose() + this->parentEntity->GetWorldPose();

  receiver_pose_.header.frame_id = "/world";
  receiver_pose_.header.stamp.sec =  (world->GetSimTime()).sec;
  receiver_pose_.header.stamp.nsec =  (world->GetSimTime()).nsec;
  receiver_pose_.pose.position.x = referencePose.pos.x;
  receiver_pose_.pose.position.y = referencePose.pos.y;
  receiver_pose_.pose.position.z = referencePose.pos.z;
  receiver_pose_.pose.orientation.w = referencePose.rot.w;
  receiver_pose_.pose.orientation.x = referencePose.rot.x;
  receiver_pose_.pose.orientation.y = referencePose.rot.y;
  receiver_pose_.pose.orientation.z = referencePose.rot.z;
  


  transmitters_.header.frame_id = "/world";
  transmitters_.header.stamp.sec =  (world->GetSimTime()).sec;
  transmitters_.header.stamp.nsec =  (world->GetSimTime()).nsec;

  AoA_.header.frame_id = "/wireless_receiver"; //todo
  AoA_.header.stamp.sec =  (world->GetSimTime()).sec;
  AoA_.header.stamp.nsec =  (world->GetSimTime()).nsec;

  rss_.header.frame_id = "/wireless_receiver";
  rss_.header.stamp.sec =  (world->GetSimTime()).sec;
  rss_.header.stamp.nsec =  (world->GetSimTime()).nsec;

 
  transmitters_.channels.resize(1);
  rss_.channels.resize(1);
  AoA_.channels.resize(1);


  int count = 0;//id of the transmitter
  physics::Model_V models = this->world->GetModels();
  for(physics::Model_V::iterator it = models.begin(); it != models.end(); it++)
  {
    std::string name = (*it)->GetName();

    //ROS_INFO("-- GazeboRosWirelessReceiver::Update()::  name  == %s",name.c_str());
    
    if( name.compare(0, 15, "wireless_router") == 0 )
    {
      ROS_INFO("-- GazeboRosWirelessReceiver::Update()::  name  == %s",name.c_str());
      //std::cout << "name: " << name << std::endl;
      math::Pose model_pose = (*it)->GetWorldPose();
      math::Pose relative_pose = -(referencePose - model_pose);//router coordinator to receiver coordinator

      geometry_msgs::Point32 p;
      p.x = model_pose.pos.x;
      p.y = model_pose.pos.y;
      p.z = model_pose.pos.z;
      transmitters_.points.push_back(p);
      transmitters_.channels[0].values.push_back(count);

      double dist = relative_pose.pos.GetLength();

      double backup_dist = dist;

      dist = rss_sensor_model_(dist, dt);
      geometry_msgs::Point32 dist_p;
      dist_p.x = dist;//dist; //todo
      rss_.points.push_back(dist_p);
      rss_.channels[0].values.push_back(count);

      relative_pose.pos.Normalize();//only direction
      math::Vector3 dir  = AoA_sensor_model_(relative_pose.pos, dt);
      geometry_msgs::Point32 relative_p;
      relative_p.x = dir.x;
      relative_p.y = dir.y;
      relative_p.z = dir.z;

      AoA_.points.push_back(relative_p);
      AoA_.channels[0].values.push_back(count);
      count++;

      ROS_INFO("-- GazeboRosWirelessReceiver::Update():: dist_with_sensor_model = %lf", dist);
      ROS_INFO("-- GazeboRosWirelessReceiver::Update():: actual_dist = %lf", backup_dist);

      double noise = abs(math::Rand::GetDblNormal(0.0, 6));


    double wavelength = common::SpeedOfLight / (this->transmitterSensor->GetFreq() * 1000000);
///////// Receiver plugin part////////
  	double rxPower = this->transmitterSensor->GetPower() + this->transmitterSensor->GetGain() + this->parentSensor->GetGain() - noise +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 20 * log10(dist); //should be *n number of obstacles
      ROS_INFO("RX WITH FORMULA = %lf", rxPower);
 wifi_tools::WifiData msg;
    msg.data=int(rxPower);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/odom";

	wifi_pub.publish(msg);
    }
    
  }   
  if(count > 0)
  {
    AoA_pub_.publish(AoA_);
    transmitter_pub_.publish(transmitters_);
    rss_pub_.publish(rss_);
    receiver_pub_.publish(receiver_pose_);
  }

  
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosWirelessReceiver::OnUpdate()
{

  ROS_INFO("GazeboRosWirelessReceiver::OnUpdate()");
  std::string txEssid;
  sensors::SensorPtr _sensor;
  double rxPower;
  bool k;
  //math::Pose pose_rx;


  math::Pose pose_rx = sensor_->GetPose() + this->parentEntity->GetWorldPose();


  //pose_rx=this->parentSensor->GetPose();
  //msgs::WirelessNodes wirelessinfo;

  // k=this-parentSensor->UpdateImpl(true);
  //gazebo::sensors::WirelessTransmitter  wirelessTransmitter ;
  //sensors::WirelessTransmitterPtr  wirelessTransmitter ;
  // wirelessTransmitter=boost::dynamic_pointer_cast<sensors::WirelessTransmitter>(_sensor);
  //std::cout << pose_rx << " \n";



  //ROS_INFO("parentSensor->GetGain() = %lf",this->parentSensor->GetGain());
  //std::cout << this->parentSensor->GetGain()<< " \n";

  std::string world_name=this->parentSensor->GetWorldName();


  //wirelessTransmitter.Init();

  //wirelessTransmitter.Load(world_name);
  //ROS_INFO("wirelessTransmitter.GetGain() = %lf",wirelessTransmitter.GetGain());
  //std::cout << wirelessTransmitter.GetGain()<< " \n";

  //ROS_INFO("wirelessTransmitter.NObstacle = %lf",wirelessTransmitter.NObstacle);

  //ROS_INFO("wirelessTransmitter.IsActive() = %d",wirelessTransmitter.IsActive());

  //ROS_INFO("wirelessTransmitter.GetESSID() = %s",wirelessTransmitter.GetESSID().c_str());


  /*if(this->world->GetEntity("wireless_router") != NULL)
  {
    ROS_ERROR("got something!!!");
    physics::EntityPtr test = this->world->GetEntity("wireless_router");
    sdf::ElementPtr elementptr = test->GetSDF();
    //wirelessTransmitter.Load(world_name, elementptr);


    sensors::WirelessTransmitterPtr tx = boost::static_pointer_cast<sensors::WirelessTransmitter>(
        sensors::SensorManager::Instance()->GetSensor("wirelessTransmitter"));

    ROS_ERROR("got it yay!!!");

    ROS_INFO("tx->GetESSID() = %s",tx->GetESSID().c_str());
  }*/


  ROS_INFO("testing...");
  /*int count = 0;//id of the transmitter
  physics::Model_V models = this->world->GetModels();
  for(physics::Model_V::iterator it = models.begin(); it != models.end(); it++)
  {
    ROS_INFO("Yay, found something...");
    std::string name = (*it)->GetName();

    //ROS_INFO("-- GazeboRosWirelessReceiver::Update()::  name  == %s",name.c_str());

    if( name.compare(0, 15, "wireless_router") == 0 )
    {
      ROS_INFO("-- found sensor wireless_router");
      //std::cout << "name: " << name << std::endl;
      math::Pose model_pose = (*it)->GetWorldPose();

      ROS_ERROR("-- for: getting the WirelessTransmitter");
      // Get the parent sensor.
      this->transmitterSensor =
          boost::dynamic_pointer_cast<sensors::WirelessTransmitter>(*it);

      // Make sure the parent sensor is valid.
      if (!this->transmitterSensor)
      {
        gzerr << "WirelessTransmitterPlugin requires a WirelessTransmitter.\n";
        return;
      }
      ROS_ERROR("-- GazeboRosWirelessReceiver::Load: got the WirelessTransmitter");

    }

  }*/





  ROS_INFO("before getting the rxPower...");
  rxPower = this->transmitterSensor->GetSignalStrength(pose_rx,this->parentSensor->GetGain());
  //rxPower =this->transmitterSensor->GetSignalStrength(this->parentSensor->GetPose(), this->parentSensor->GetGain());


  ROS_INFO("after getting the rxPower...");

  ROS_INFO("pose_rx: x = %lf | y = %lf | z = %lf",pose_rx.pos.x, pose_rx.pos.y, pose_rx.pos.z);
  ROS_INFO("rxPower = %lf\n\n",rxPower);

  /*wirelessTransmitter.GetGain();
  //std::cout << rxPower << " \n";
  ROS_INFO("rxPower = %lf",rxPower);
  ROS_INFO("GazeboRosWirelessReceiver::OnUpdate(): End reached\n");*/
this->node_handle_ = new ros::NodeHandle(namespace_);
    ros::NodeHandle n1;
    //ros::Publisher chatter_pub = n1.advertise<wifi_tools::WifiData>("/wifi_tools/wifi_data", 1);
	/*ros::Publisher chatter_pub = this->node_handle_->advertise<std_msgs::Int16>("proba1", 1);
 	ros::Publisher chatter_pub2 = n1.advertise<std_msgs::Int16>("proba2", 1);
    wifi_tools::WifiData msg;
    msg.data=int(rxPower);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/odom";
    //std_msgs::Int16 msg;
   msg.data =6;
	chatter_pub.publish(msg);
	chatter_pub2.publish(msg);
	chatter_pub3.publish(msg);
    //ros::spinOnce();

    if (!ros::isInitialized()) 
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "gazebo_ros_wireless_receiver", ros::init_options::NoSigintHandler);
        }
        this->node_handle_ = new ros::NodeHandle("wifi");
        chatter_pub = this->node_handle_->advertise<wifi_tools::WifiData>("/wifi_tools/wifi_data", 1,true);

        wifi_tools::WifiData msg;
    	msg.header.stamp = ros::Time::now();
    	msg.header.frame_id = "/odom";
    	msg.data =int(rxPower);
        chatter_pub.publish(msg);
    	//ros::spinOnce();*/
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosWirelessReceiver)

} // namespace gazebo



