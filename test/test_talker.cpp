/**
 * @file test_talker.cpp
 * @author Sameer Pusegaonkar (sameer@umd.edu)
 * @brief Add a test file to test the talker node
 * @version 0.1
 * @date 2021-11-12
 * @copyright Copyright (c) 2021
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <string>

#include "beginner_tutorials/CheckString.h"


/**
 * @brief Test case to check the existence and success of calling
 * the ChangeStr service 
 * @param none
 * @return none
 */
std::shared_ptr<ros::NodeHandle> nh;
TEST(TESTSuite, ChangeStrSrv) {
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::CheckString>(
      "CheckString");

  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::CheckString srv;
  srv.request.input_msg = "INFO";
  client.call(srv);

  EXPECT_EQ(srv.response.output_res, "Robot is alright!");
}

TEST(TESTSuite, transformTest) {
  // Create TF listener object
  tf::TransformListener listener;
  // Create transform object to store the published transform
  tf::StampedTransform transform;
  // Wait till the transform is published
  if (listener.waitForTransform("world", "talk", ros::Time(0),
                                ros::Duration(100))) {
    // Get the value of the published transform
    listener.lookupTransform("world", "talk", ros::Time(0), transform);

    // Check the values of the published transform
    EXPECT_EQ(1.0, transform.getOrigin().x());
    EXPECT_EQ(1.0, transform.getOrigin().x());
    EXPECT_EQ(1.0, transform.getOrigin().x());
  }
}


/**
 * @brief Run all rostests for talker node
 * @param none
 * @return 0 on successful exit
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "check_output");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}