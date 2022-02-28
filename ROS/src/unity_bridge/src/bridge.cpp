#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "ovis_msgs/OvisJoints.h"

constexpr size_t JOINTS_COUNT = 7;

void targetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard: [%e, %e, %e]", msg->position.x, msg->position.y, msg->position.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unity_bridge");

  ros::NodeHandle n;

  ros::Publisher joints_pub = n.advertise<ovis_msgs::OvisJoints>("ovis_joints", 1000);
  ros::Subscriber sub = n.subscribe<geometry_msgs::Pose>("ovis_target", 1000, targetCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ovis_msgs::OvisJoints msg;
    msg.header.seq = count;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "ovis";

    std::vector<std::string> joint_names;
    std::vector<geometry_msgs::Vector3> eulers;

    for (size_t i = 0; i < JOINTS_COUNT; i++)
    {
      joint_names.push_back("name" + std::to_string(i));

      geometry_msgs::Vector3 euler;
      euler.x = 0;
      euler.y = 0;
      euler.z = 0;
      eulers.push_back(euler);
    }

    geometry_msgs::Vector3 testEuler;
    testEuler.x = 0;
    testEuler.y = 0;
    testEuler.z = count;
    eulers[0] = testEuler;

    msg.joint_names = joint_names;
    msg.eulers = eulers;

    joints_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}
