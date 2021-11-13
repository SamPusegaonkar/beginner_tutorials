/**
 * @file talker.cpp
 * @author Sameer Pusegaonkar (sameer@umd.edu)
 * @brief A talker file to print out the message collected at a topic. It also has a ROSService
 * @version 0.1
 * @date 2021-10-29
 * @copyright Copyright (c) 2021
 */

#include <sstream>
#include <tf/transform_broadcaster.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/CheckString.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

/**
 * @brief A ROS service method to check if the robot message state is valid
 * @param req This is a request sent by a client
 * @param res This is a response sent to the client
 * @return true If the robot state is valid
 * @return false If the robot state is false
 */
bool CheckForValidMessage(beginner_tutorials::CheckString::Request  &req,
          beginner_tutorials::CheckString::Response &res) {
  if ( req.input_msg == "INFO" ) {
    ROS_INFO_STREAM("Request to check if robot is ok");
    ROS_INFO_STREAM("Sending back response: Robot is alright!");
    res.output_res = "Robot is alright!";

  } else if ( req.input_msg == "DEBUG" ) {
    ROS_DEBUG_STREAM("Debugging errors");
    ROS_DEBUG_STREAM("Sending back response: This is a debugging message");
    ROS_DEBUG_STREAM("INSIDE DEBUGGING");
    res.output_res = "The programmer is currently debugging";

  } else if ( req.input_msg == "WARN" ) {
    ROS_WARN_STREAM("Request to check if robot is giving a warning");
    ROS_WARN_STREAM("sending back response: Robot is returning some warnings");
    res.output_res = "Robot is returning some warnings!";

  } else if ( req.input_msg == "ERROR" ) {
    ROS_ERROR_STREAM("Request to check if the robot is giving a warning");
    ROS_ERROR_STREAM("Sending back response: Robot returned some errors!");
    res.output_res = "Robot has some errors";

  } else if ( req.input_msg == "FATAL" ) {
    ROS_FATAL_STREAM("Request to check if a robot is giving any fatal errors");
    ROS_FATAL_STREAM("Sending back response: Robot is on fire!!!");
    res.output_res = "Robot is on fire!!!!!!!!!!!!!!";
  }
  return true;
}


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

  ros::ServiceServer service = n.advertiseService("CheckString", CheckForValidMessage);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(1, 1, 1));
  tf::Quaternion q;
  q.setRPY(1, 1, 1);
  transform.setRotation(q);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Hi, This is Sameer's Robot";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
