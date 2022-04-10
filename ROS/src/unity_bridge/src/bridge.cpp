#include "ros/ros.h"
#include "ovis_msgs/OvisJointAngles.h"
#include "ovis_msgs/OvisJointGoal.h"
#include "ovis_msgs/OvisIKGoal.h"
#include "ovis_msgs/HomeJoint.h"

constexpr size_t JOINTS_COUNT = 6;

enum class ArmState { Idle, JointByJoint, IKGoal };

ArmState armState;
ovis_msgs::OvisJointGoal goal;

std::vector<float> joint_angles;

// void targetCallback(const geometry_msgs::Pose::ConstPtr& msg)
// {
//     ROS_INFO("I heard: [%e, %e, %e]", msg->position.x, msg->position.y, msg->position.z);
// }

void OvisJointGoalCallback(const ovis_msgs::OvisJointGoal::ConstPtr& msg)
{
    ROS_INFO("I heard OvisJointGoalCallback: [%d, %e]", msg->joint_index, msg->joint_angle);

    goal.joint_index = msg->joint_index;
    goal.joint_angle = msg->joint_angle;

    armState = ArmState::JointByJoint;
}

bool HomeJointCallback(ovis_msgs::HomeJointRequest &request, ovis_msgs::HomeJointResponse &response)
{
    for (size_t i = 0; i < JOINTS_COUNT; i++)
    {
        response.home_joint_positions.push_back(0);
    }
    response.home_joint_positions[1] = 180;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unity_bridge");

    ros::NodeHandle n;

    ros::ServiceServer home_srv = n.advertiseService("ovis/home_joint_positions", HomeJointCallback);

    ros::Publisher joints_pub = n.advertise<ovis_msgs::OvisJointAngles>("ovis/joint_angles", 1);
    
    //JointByJoint topic
    ros::Subscriber jgoal_sub = n.subscribe<ovis_msgs::OvisJointGoal>("ovis/joint_goal", 100, OvisJointGoalCallback);

    // IKGoal topic
    // n.subscribe<geometry_msgs::Pose>("ovis_target", 1000, targetCallback);
    
    armState = ArmState::Idle;
    for (size_t i = 0; i < JOINTS_COUNT; i++)
    {
        joint_angles.push_back(0);
    }
    joint_angles[1] = 180;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        //Check for new messages
        ros::spinOnce();

        switch (armState)
        {
        case ArmState::JointByJoint:
            joint_angles[goal.joint_index] = goal.joint_angle;
            break;
        }

        ovis_msgs::OvisJointAngles joint_msg;
        joint_msg.joint_angles = joint_angles;
        joints_pub.publish(joint_msg);

        loop_rate.sleep();
    }

    return 0;
}
