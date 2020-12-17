/***********************************************************************************************************************
 *
 * Copyright (c) 2018, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <abb_libegm/egm_controller_interface.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>

trajectory_msgs::MultiDOFJointTrajectoryPoint set_point;
bool new_set_point_received = false;

void setpoint_callback(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg )
{
    set_point.transforms.operator=(msg.transforms);
    set_point.velocities.operator=(msg.velocities);
    new_set_point_received = true;
}

int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "pose_velocity_controller_node");
  //TODO: multithreading for callback
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // OPTIONS
  bool use_vel = false;
  int egm_port = 6510;

  ros::Publisher feedback_publisher = node_handle.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/robot_feedback", 1);
  ros::Subscriber egm_sub = node_handle.subscribe("/robot_set_point", 1, &setpoint_callback);

  // Boost components for managing asynchronous UDP socket(s).
  boost::asio::io_service io_service;
  boost::thread_group thread_group;

  // Create EGM configurations.
  abb::egm::BaseConfiguration configuration;
  //TODO: change with position only
  configuration.use_velocity_outputs = use_vel;

  // Create an EGM interface:
  // * Sets up an EGM server (that the robot controller's EGM client can connect to).
  // * Provides APIs to the user (for setting motion references, that are sent in reply to the EGM client's request).
  //
  // Note: It is important to set the correct port number here,
  //       as well as configuring the settings for the EGM client in thre robot controller.
  //       If using the included RobotStudio Pack&Go file, then port 6511 = ROB_1, 6512 = ROB_2, etc.
  abb::egm::EGMControllerInterface egm_interface(io_service, egm_port, configuration);

  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  //----------------------------------------------------------
  // Execute a pose velocity controller loop.
  //
  // Note 1: The EGM communication session is started by the
  //         EGMRunPose RAPID instruction.
  //
  // Note 2: To get pure velocity control, then the EGM client
  //         (in the robot controller) need its position
  //         correction gain to be set to 0. This is done with
  //         the EGMRunPose RAPID instruction.
  //----------------------------------------------------------
  ROS_INFO("========== Pose velocity controller (open-loop) sample ==========");
  bool wait = true;
  abb::egm::wrapper::Input input;
  abb::egm::wrapper::CartesianVelocity initial_velocity;
  abb::egm::wrapper::CartesianPose initial_pose;


  const int egm_rate = 250.0; // [Hz] (EGM communication rate, specified by the EGMActPose RAPID instruction).
  int sequence_number = 0;    // [-] (sequence number of a received EGM message).
  double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).

  abb::egm::wrapper::Output output;

  ROS_INFO("1: Wait for an EGM communication session to start...");
  while(ros::ok() && wait)
  {
    if(egm_interface.isConnected())
    {
      if(egm_interface.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }

    ros::Duration(0.5).sleep();
  }

  while(ros::ok())
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
    if(egm_interface.waitForMessage(500))
    {
      // Read the message received from the EGM client.
      egm_interface.read(&input);
      sequence_number = input.header().sequence_number();

      if(sequence_number == 0)
      {
        // Reset all references, if it is the first message.
        output.Clear();
        initial_velocity.CopyFrom(input.feedback().robot().cartesian().velocity());
        output.mutable_robot()->mutable_cartesian()->mutable_velocity()->CopyFrom(initial_velocity);
        initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
        output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
      }
      else
      {
        time = sequence_number/((double) egm_rate);

        // get position feedback
        geometry_msgs::Twist feedback_vel;
        geometry_msgs::Transform feedback_pos;
        feedback_pos.translation.x = input.feedback().robot().cartesian().pose().position().x();
        feedback_pos.translation.y = input.feedback().robot().cartesian().pose().position().y();
        feedback_pos.translation.z = input.feedback().robot().cartesian().pose().position().z();
        //TODO: check quaternion order
        feedback_pos.rotation.w = input.feedback().robot().cartesian().pose().quaternion().u0();
        feedback_pos.rotation.x = input.feedback().robot().cartesian().pose().quaternion().u1();
        feedback_pos.rotation.y = input.feedback().robot().cartesian().pose().quaternion().u2();
        feedback_pos.rotation.z = input.feedback().robot().cartesian().pose().quaternion().u3();

        // get velocity feedback
        feedback_vel.linear.x = input.feedback().robot().cartesian().velocity().linear().x();
        feedback_vel.linear.y = input.feedback().robot().cartesian().velocity().linear().y();
        feedback_vel.linear.z = input.feedback().robot().cartesian().velocity().linear().z();
        feedback_vel.angular.x = input.feedback().robot().cartesian().velocity().angular().x();
        feedback_vel.angular.y = input.feedback().robot().cartesian().velocity().angular().y();
        feedback_vel.angular.z = input.feedback().robot().cartesian().velocity().angular().z();

        // // Publish Feedback
        trajectory_msgs::MultiDOFJointTrajectoryPoint feedback_msg;
        feedback_msg.transforms.push_back(feedback_pos);
        feedback_msg.velocities.push_back(feedback_vel);
        feedback_publisher.publish(feedback_msg);
        
        // If new set point received from the call back function, apply it to the controller
        if (new_set_point_received)
        {
          new_set_point_received = false;
          // Set Cartesian Position
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(set_point.transforms[0].translation.x);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_y(set_point.transforms[0].translation.y);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_z(set_point.transforms[0].translation.z);
          //TODO: check quaternion order
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u0(set_point.transforms[0].rotation.w);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u1(set_point.transforms[0].rotation.x);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u2(set_point.transforms[0].rotation.y);
          output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u3(set_point.transforms[0].rotation.z);

          // // Set Cartesian Velocity
          output.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear()->set_x(set_point.velocities[0].linear.x);
          output.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear()->set_y(set_point.velocities[0].linear.y);
          output.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear()->set_z(set_point.velocities[0].linear.z);
          output.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_angular()->set_x(set_point.velocities[0].angular.x);
          output.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_angular()->set_y(set_point.velocities[0].angular.y);
          output.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_angular()->set_z(set_point.velocities[0].angular.z);
        }
        if(sequence_number%egm_rate == 0)
        {
          // TODO print feedback
          ROS_INFO("Connected");
          // ROS_INFO_STREAM("y current   =   " << feedback_msg.transforms[0].translation.x);
          // ROS_INFO_STREAM("z current   =   " << feedback_msg.transforms[0].translation.x);
          // ROS_INFO_STREAM("q1 current  =   " << feedback_msg.transforms[0].rotation.w);
          // ROS_INFO_STREAM("q2 current  =   " << feedback_msg.transforms[0].rotation.x);
          // ROS_INFO_STREAM("q3 current  =   " << feedback_msg.transforms[0].rotation.y);
          // ROS_INFO_STREAM("q4 current  =   " << feedback_msg.transforms[0].rotation.z);
        }
      }

      // Write references back to the EGM client.
      egm_interface.write(output);
    }
  }

  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}
