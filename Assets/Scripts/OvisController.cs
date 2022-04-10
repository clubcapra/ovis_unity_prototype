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
    public string rosNameSpace = "/markhor/";
    public string topicJointAngles = "ovis/joint_angles";
    public string topicJointGoal = "ovis/joint_goal";

    public string serviceHomePos = "ovis/home_joint_positions";

    public Action<HomeJointResponse> OnHomeReceived;

    public JointController[] joints;
    public ROSConnection rosConn;

    public float updateTime = 0.5f;

    private float lastUpdate = 0;

    void Awake()
    {
        if(rosConn == null)
            rosConn = GetComponent<ROSConnection>();

        lastUpdate = Time.realtimeSinceStartup;
    }

    private void OnEnable()
    {
        Debug.Log("enalbe ou whatever la kekchose de there you go easy");
        rosConn.Connect();

        topicJointAngles = rosNameSpace + topicJointAngles;
        topicJointGoal = rosNameSpace + topicJointGoal;
        serviceHomePos = rosNameSpace + serviceHomePos;

        rosConn.RegisterRosService<HomeJointRequest, HomeJointResponse>(serviceHomePos);

        rosConn.SendServiceMessage<HomeJointResponse>(serviceHomePos, new HomeJointRequest(), OnHomePositionsReceived);

        rosConn.RegisterPublisher<OvisJointGoalMsg>(topicJointGoal, 1);
    }

    public void SendJointGoal(OvisJointGoalMsg jointGoal)
    {
        Debug.Log($"SendJointGoal {jointGoal.joint_index}, {jointGoal.joint_angle}");

        rosConn.Publish(topicJointGoal, jointGoal);
        lastUpdate = Time.realtimeSinceStartup;
    }

    private void OnHomePositionsReceived(HomeJointResponse res)
    {
        Debug.Log("OnHomePositionsReceived");

        if (res.home_joint_positions.Length != joints.Length)
        {
            Debug.LogError($"OnHomePositions has {res.home_joint_positions.Length} joints but only {joints.Length} in the Editor");
            return;
        }

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].SetHomePositionOffset(res.home_joint_positions[i]);
        }

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].SetAngularPosition(res.current_joint_positions[i]);
        }

        rosConn.Subscribe<OvisJointAnglesMsg>(topicJointAngles, OnJointAnglesReceived);

        OnHomeReceived?.Invoke(res);
    }

    private void OnJointAnglesReceived(OvisJointAnglesMsg msg)
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

    private void OnDisable()
    {
        rosConn.Disconnect();
    }
}
