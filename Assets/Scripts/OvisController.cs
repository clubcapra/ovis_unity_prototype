using RosMessageTypes.Geometry;
using RosMessageTypes.Ovis;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class OvisController : MonoBehaviour
{
    //https://docs.google.com/document/d/1EtxCm3eE4_atR6cnkAefFj6paXdG1T3sMXU4caz4XTg/edit

    public enum ControlMode { JointByJoint, Target }

    public ControlMode controlMode = ControlMode.JointByJoint;

    public string topicJointAngles = "ovis/joint_angles";
    public string serviceHomePos = "ovis/home_joint_positions";

    public JointController[] joints;

    private ROSConnection rosConn;

    private JointGoalController jointGoalController;

    void Awake()
    {
        rosConn = GetComponent<ROSConnection>();

        jointGoalController = GetComponent<JointGoalController>();
        jointGoalController.enabled = false;
    }

    private void OnEnable()
    {
        rosConn.Connect();

        rosConn.SendServiceMessage<HomeJointResponse>(serviceHomePos, new HomeJointRequest(), OnHomePositions);
    }

    private void OnHomePositions(HomeJointResponse res)
    {
        if (res.home_joint_positions.Length != joints.Length)
        {
            Debug.LogError($"OnHomePositions has {res.home_joint_positions.Length} joints but only {joints.Length} in the Editor");
            return;
        }

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].SetHomePositionOffset(res.home_joint_positions[i]);
        }

        rosConn.Subscribe<OvisJointAnglesMsg>(topicJointAngles, OnReceiveJointAngles);

        //jointGoalController.Prepare(rosConn, res);

        OnValidate();
    }

    private void OnReceiveJointAngles(OvisJointAnglesMsg msg)
    {
        if (msg.joint_angles.Length != joints.Length)
        {
            Debug.LogError($"OvisJointAnglesMsg has {msg.joint_angles.Length} joints but only {joints.Length} in the Editor");
            return;
        }

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].SetAngularPosition(msg.joint_angles[i]);
        }
    }

    private void OnValidate()
    {
        if(jointGoalController != null)
            jointGoalController.enabled = controlMode == ControlMode.JointByJoint;
    }

    private void OnDisable()
    {
        jointGoalController.enabled = false;

        rosConn.Disconnect();
    }
}
