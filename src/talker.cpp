/**
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

 /**
 * @file      talker.cpp
 * @author    Rohan Singh
 * @copyright Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * @brief     Publisher node
 */

#include <sstream>
#include <functional>

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
bool concatStringsCallback(beginner_tutorials::ConcatStrings::Request &req,
       beginner_tutorials::ConcatStrings::Response &res) {
  /* Add the 2 strings */
  res.resultString = req.first + req.second;
  ROS_INFO("request: first=%s, second=%s",
             req.first.c_str(), req.second.c_str());
  ROS_INFO("sending back response: [%s]", res.resultString.c_str());
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

  ros::Rate loopRate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    msg.data = str;

    ROS_INFO("%s", msg.data.c_str());

    /* Publish updated message */
    chatterPub.publish(msg);

    ros::spinOnce();

    loopRate.sleep();
    ++count;
  }
  return 0;
}

