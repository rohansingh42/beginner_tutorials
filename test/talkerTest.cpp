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
 * @file      talkerTest.cpp
 * @author    Rohan Singh
 * @copyright MIT License
 * @brief     Test file for talker node
 */

#include <gtest/gtest.h>

#include "ros/ros.h"
#include "ros/service_client.h"
#include "beginner_tutorials/ConcatStrings.h"

/**
 * @brief Test to check functionality of service ConcatStrings
 */
TEST(TESTSuite, TestConcatStrings) {
  /* Create node handle */
  ros::NodeHandle n;

  /* Create service client */
  ros::ServiceClient client = n.serviceClient
    <beginner_tutorials::ConcatStrings>("concatStringService");

  /* Check if sevice exists */
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  /* Test output of service */
  beginner_tutorials::ConcatStrings srv;
  srv.request.first = "Hello ";
  srv.request.second = "World";
  client.call(srv);
  EXPECT_STREQ("Hello World", srv.response.resultString.c_str());
}
