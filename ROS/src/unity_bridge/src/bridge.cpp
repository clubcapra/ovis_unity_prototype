#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "ovis_msgs/OvisJoints.h"

constexpr size_t JOINTS_COUNT = 7;

void targetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard: [%e, %e, %e]", msg->position.x, msg->position.y, msg->position.z);
}

void simCallback(const ovis_msgs::OvisJoints::ConstPtr& msg)
{
  ROS_INFO("I heard: [%e, %e, %e, %e, %e, %e, %e]", 
    msg->joint_agnles[0], 
    msg->joint_agnles[1], 
    msg->joint_agnles[2],
    msg->joint_agnles[3],
    msg->joint_agnles[4],
    msg->joint_agnles[5],
    msg->joint_agnles[6]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unity_bridge");

  ros::NodeHandle n;

  //Need this when controlling the arm from Unity
  n.subscribe<ovis_msgs::OvisJoints>("sim_joints", 1000, simCallback);

  //Need this when using the IK on the robot arm
  n.subscribe<geometry_msgs::Pose>("ovis_target", 1000, targetCallback);
  ros::Publisher joints_pub = n.advertise<ovis_msgs::OvisJoints>("ovis_joints", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ovis_msgs::OvisJoints msg;
    msg.header.seq = count;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "ovis";

    std::vector<std::string> joint_names;
    std::vector<float> joint_agnles;

    for (size_t i = 0; i < JOINTS_COUNT; i++)
    {
      joint_names.push_back("name" + std::to_string(i));
      joint_agnles.push_back(0);
    }

    //For test purpose only
    joint_agnles[0] = count;

    msg.joint_names = joint_names;
    msg.joint_agnles = joint_agnles;

    joints_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}
