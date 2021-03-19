/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// global publisher to forward gazebo msgs to ROS
ros::Publisher ros_forward_pub;

/////////////////////////////////////////////////
// Function is called every time a message is received.
void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
    if (name == std::string("box"))
    {
      const auto &position = pose.position();
      const auto &orientation = pose.orientation();

      geometry_msgs::PoseStamped ros_pose;
      ros_pose.pose.position.x = position.x();
      ros_pose.pose.position.y = position.y();
      ros_pose.pose.position.z = position.z();
      ros_pose.pose.orientation.x = orientation.x();
      ros_pose.pose.orientation.y = orientation.y();
      ros_pose.pose.orientation.z = orientation.z();
      ros_pose.pose.orientation.w = orientation.w();

      ros_pose.header.frame_id = 1;
      ros_pose.header.stamp = ros::Time::now();
      ros_forward_pub.publish(ros_pose);

    }
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);
  ros::init(_argc, _argv, "target_tracker");

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  ros::NodeHandle nh;
  ros_forward_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("hawkeye/target_pose", 10);

  // Listen to Gazebo pose info topic
  gazebo::transport::SubscriberPtr sub =
  node->Subscribe("~/pose/info", posesStampedCallback);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
