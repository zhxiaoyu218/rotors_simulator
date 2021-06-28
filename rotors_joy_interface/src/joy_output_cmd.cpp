/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "rotors_joy_interface/joy.h"

#include <mav_msgs/default_topics.h>

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <thread>
#include <chrono>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
struct my_Axes {
  int roll;
  int pitch;
  int thrust;
  int yaw;
  int roll_direction;
  int pitch_direction;
  int thrust_direction;
  int yaw_direction;
};
struct my_Buttons {
  int unpause;
  int force;
  int takeoff;
  int land;
  int ctrl_enable;
  int ctrl_mode;
  int yaw_left;
  int yaw_right;
};
my_Axes my_axes;
my_Buttons my_button;
ros::ServiceClient client;

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  
  pnh.param("axis_roll_", axes_.roll, 0);
  pnh.param("axis_pitch_", axes_.pitch, 1);
  pnh.param("axis_thrust_", axes_.thrust, 2);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 10.0 * M_PI / 180.0);  // [rad]
  // pnh.param("max_yaw_rate", max_.rate_yaw, 45.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_yaw_rate", max_.rate_yaw, 60.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]

  pnh.param("v_yaw_step", v_yaw_step_, 0.05);  // [rad/s]

  pnh.param("is_fixed_wing", is_fixed_wing_, false);

  pnh.param("button_yaw_left_", buttons_.yaw_left, 3);
  pnh.param("button_yaw_right_", buttons_.yaw_right, 4);
  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 5);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 10);
  pnh.param("button_takeoff_", buttons_.takeoff, 7);
  pnh.param("button_land_", buttons_.land, 8);


  pnh.param("myaxis_roll", my_axes.roll, 3);
  pnh.param("myaxis_pitch", my_axes.pitch, 4);
  pnh.param("myaxis_thrust", my_axes.thrust, 1);
  pnh.param("myaxis_yaw", my_axes.yaw, 0);
  pnh.param("myaxis_direction_roll", my_axes.roll_direction, -1);
  pnh.param("myaxis_direction_pitch", my_axes.pitch_direction, 1);
  pnh.param("myaxis_direction_thrust", my_axes.thrust_direction, 1);
  pnh.param("myaxis_direction_yaw", my_axes.yaw_direction, 1);
  pnh.param("button_unpause", my_button.unpause, 4);
  pnh.param("button_force", my_button.force, 3);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  // std::cout << "joy " << std::endl;

  control_msg_.roll = msg->axes[my_axes.roll] * max_.roll * my_axes.roll_direction;
  control_msg_.pitch = msg->axes[my_axes.pitch] * max_.pitch * my_axes.pitch_direction - 0.041;

  control_msg_.thrust.z = (0.3 * msg->axes[my_axes.thrust]) * max_.thrust / 2.0 * my_axes.thrust_direction +  14.85;
  control_msg_.yaw_rate = (msg->axes[my_axes.yaw]/0.18) * max_.rate_yaw * my_axes.yaw_direction; // max is 0.179...

  // std::cout << "control_msg_.roll: " << control_msg_.roll << "control_msg_.pitch: " << control_msg_.pitch << std::endl;
  // std::cout << "control_msg_.thrust.z: " << control_msg_.thrust.z << "control_msg_.yaw_rate: " << control_msg_.yaw_rate << std::endl;
  // // std::cout << "T: " << (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction << std::endl;
  // std::cout << "unpause: " << msg->buttons[my_button.unpause] << std::endl;

  if(msg->buttons[my_button.unpause])
  {
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused) {
      ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      unpaused = ros::service::call("/gazebo/unpause_physics", srv);
      ++i;
    }
    if(unpaused)
      ROS_INFO("unpause the drone! ");
    else
      ROS_INFO("unpause fail! ");
  }
  if(msg->buttons[my_button.force])
  {
    gazebo_msgs::ApplyBodyWrench srv;
    srv.request.body_name = "hummingbird::hummingbird/base_link";
    geometry_msgs::Point P;
    P.x = 0.0;
    P.y = 0.0;
    P.z = 0.0;
    srv.request.reference_point = P;
    // srv.request.start_time = ros::Time;
    srv.request.wrench.force.x = 10.0;
    srv.request.wrench.force.y = 0.0;
    srv.request.wrench.force.z = 0.0;
    srv.request.duration.nsec = 100000000;

    client.call(srv);
    // bool unpaused = ros::service::call("/gazebo/apply_body_wrench '{body_name: "hummingbird::hummingbird/base_link",reference_point: {x: 0.0, y: 0.0, z: 0.0}, wrench:{ force: {x: 10, y: 0.0, z: 0.0}}, start_time: 0, duration: 1000000000}'", srv);
   
    // if(unpaused)
      ROS_INFO("apply force ! ");
    // else
    //   ROS_INFO("apply force fail! ");
  }

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Joy joy;

  ros::spin();

  return 0;
}
