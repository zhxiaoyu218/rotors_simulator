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

#include <thread>

#include <Eigen/Core>
#include <ros/ros.h>

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/Imu.h>
#include "std_msgs/Int8.h"
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry> 
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>


ros::ServiceClient client;
sensor_msgs::Imu disturbance;

int32_t ts = 10;
Eigen::Vector3d Force1(0.745319, 0, 0), Force2(0.0, 0.745319, 0), Torque(0.0, 0.0, 0.0);
geometry_msgs::Point P;
// 1.1179785  //0.745319 // 1.490638  // 0.3726595 //1.8632975

Eigen::Vector3d position(0.0,0.0,1.0);


void odometryTransCallback(const nav_msgs::Odometry& msg){
  // use RotorS sim
  position(0) = msg.pose.pose.position.x;
  position(1) = msg.pose.pose.position.y;
  position(2) = msg.pose.pose.position.z;

}



int main(int argc, char** argv) {
  ros::init(argc, argv, "disturbance_manager");
  ros::NodeHandle nh;
  //ros::Subscriber disturbance_sub = nh.subscribe("disturbance/cmd", 1, &DisturbanceCmdCallback);
  client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  //ros::Publisher disturbance_pub = nh.advertise<sensor_msgs::Imu>("disturbance/apply", 100);
  ros::Subscriber  odom_sub  = nh.subscribe("/hummingbird/ground_truth/odometry", 1, &odometryTransCallback);



  ROS_INFO("Started disturbance_manager.");

  ros::Rate rate(500);
  ros::Time time_begin_ = ros::Time::now();
  ros::Time time_now =  ros::Time::now();
  std::string force1_frame = "world";
  ros::Duration(ts).sleep();
  while(ros::ok())
  {
    ros::spinOnce();   // 0.002
    time_now = ros::Time::now();

    // the wind range:   x (-5,5)  y(1,3)

    //std::cout << "The position is " << position << std::endl;

    gazebo_msgs::ApplyBodyWrench srv;
    if( position(0)<5.0 && position(0)>1.0 && position(1)<5.0 && position(1)>-5.0  )
    {

      ROS_INFO("Apply the external force1 !!!");
      std::cout << "The position is " << position << std::endl;

      std::cout << "The Force1 is " << Force1 << std::endl;

      srv.request.body_name = "hummingbird::hummingbird/base_link";
      // srv.request.body_name = "ardrone::ardrone/base_link";
      P.x = 0.;
      P.y = 0.;
      P.z = 0.;
      srv.request.reference_point = P;
      // srv.request.start_time = ros::Time;
      srv.request.wrench.force.x = Force1(0);
      srv.request.wrench.force.y = Force1(1);
      srv.request.wrench.force.z = Force1(2);
      srv.request.wrench.torque.x = Torque(0);
      srv.request.wrench.torque.y = Torque(1);
      srv.request.wrench.torque.z = Torque(2);
      srv.request.duration.sec = 1.0;//100000000; //nsec
      //srv.request.reference_frame = "world"; //world(inertial) hummingbird/base_link same result ？？？？TODO change the force1 and torque to body frame  //now it is world frame direction at the body point

      client.call(srv);


    }
 
    rate.sleep();
  }

  return 0;
}
