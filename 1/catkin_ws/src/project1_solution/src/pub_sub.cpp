#include "project1_solution/TwoInts.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

ros::Publisher pubber;

void CallBack(const project1_solution::TwoInts &msg) {
  std_msgs::Int16 pubmsg;
  pubmsg.data = msg.a + msg.b;
  ROS_INFO("Received %d and %d, sum is %d", msg.a, msg.b, pubmsg.data);
  pubber.publish(pubmsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pub_sub");
  ros::NodeHandle n;
  pubber = n.advertise<std_msgs::Int16>("sum", 10);
  ros::Subscriber subber = n.subscribe("two_ints", 10, CallBack);
  ros::spin();

  return 0;
}
