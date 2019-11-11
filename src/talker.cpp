/**
 *  MIT License
 *
 *  Copyright (c) 2019 Rohan Singh
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

 /**
 * @file      talker.cpp
 * @author    Rohan Singh
 * @copyright MIT License
 * @brief     Publisher node
 */

#include <sstream>
#include <functional>

#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ConcatStrings.h"

extern std::string str = "Default String";

/**
 * @brief Callback for service ConcatStrings
 *
 * @param req Requested data from service
 * @param res Response for service
 *
 * @return Returns true on execution
 */
bool concatStringsCallback(
       beginner_tutorials::ConcatStrings::Request &req,       // NOLINT
       beginner_tutorials::ConcatStrings::Response &res) {    // NOLINT
  if (req.first.empty() || req.second.empty()) {
    ROS_WARN_STREAM("Atleast one input string is empty.");
  }
  /* Add the 2 strings */
  res.resultString = req.first + req.second;
  ROS_INFO_STREAM("Request: first = " << req.first.c_str());
  ROS_INFO_STREAM("Request: second = " << req.second.c_str());
  ROS_INFO_STREAM("Sending back response: " << res.resultString.c_str());
  str = res.resultString;
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /* tf Broadcaster Object */
  static tf::TransformBroadaster br;
  
  /* Trnsformation object */
  tf::Transform transform;

  /* Set transformation matrix */
  transform.setOrigin( tf::Vector3(1.0, 1.0, 1.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 90);
  transform.setRotation(q);

  /* Define default publisher frequency */
  int pubHz = 20;

  /* Check for input arguments */
  if (argc > 2) {
    ROS_WARN_STREAM("Too many input arguments."
                     << "Considering only first argument.");
  } else if (argc == 2) {
    pubHz = atoi(argv[1]);
    if (pubHz <= 0) {
      ROS_ERROR_STREAM("Incorrect value for publisher frequency.");
      pubHz = 20;
    }
  } else {
    ROS_WARN_STREAM("Using default publisher frequency.");
  }

  ROS_DEBUG_STREAM("Publisher frequency set to : " << pubHz);
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /* Publisher for concatenated string */
  ros::Publisher chatterPub = n.advertise<std_msgs::String>("chatter", 1000);

  /* Server for ConcatStrings Service */
  ros::ServiceServer concatStringService =
         n.advertiseService("concatStringService", concatStringsCallback);

  ros::Rate loopRate(pubHz);

  if (!ros::ok()) {
    ROS_FATAL_STREAM("ROS node is not running.");
  }
  while (ros::ok()) {
    if (pubHz < 10) {
      ROS_WARN_STREAM("Publishing frequency too low.");
    }
    /* String Message object */
    std_msgs::String msg;

    msg.data = str;

    ROS_INFO_STREAM("Concatenated String : " << msg.data.c_str());

    /* Publish updated message */
    chatterPub.publish(msg);

    /* Broadast transformation */
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loopRate.sleep();
  }
  return 0;
}

