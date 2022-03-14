using RosMessageTypes.Ovis;
using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[RequireComponent(typeof(OvisController))]
public class JointGoalController : MonoBehaviour
{
    public string topicJointGoal = "ovis/joint_goal";

    public JointController[] ghostJoints;

    public int jointIndex = 0;

    private ROSConnection rosConn;

    public void Prepare(ROSConnection conn, HomeJointResponse res)
    {
        rosConn = conn;

        rosConn.RegisterPublisher<OvisJointGoalMsg>(topicJointGoal);

        for (int i = 0; i < ghostJoints.Length; i++)
        {
            ghostJoints[i].SetHomePositionOffset(res.home_joint_positions[i]);
        }
    }

    // Update is called once per frame
    void Update()
    {
        var msg = new OvisJointGoalMsg 
        {
            joint_index = (byte)jointIndex,
            joint_angle = ghostJoints[jointIndex].GetAngularPosition()
        };

        rosConn.Publish(topicJointGoal, msg);
    }
}
