using RosMessageTypes.Geometry;
using RosMessageTypes.Ovis;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class OvisController : MonoBehaviour
{
    [Header("Unity")]
    public string topicSim = "/sim_joints";

    [Header("Ovis")]
    public string topicJoints = "/ovis_joints";
    public string topicTarget = "/ovis_target";

    public Transform target;
    public Transform[] joints;

    public float updatePerSec = 1f;

    private ROSConnection rosConn;
    private string[] jointNames;
    private float[] jointAngles;

    private float lastUpdate = 0;

    // Start is called before the first frame update
    void Awake()
    {
        rosConn = GetComponent<ROSConnection>();

        jointNames = new string[joints.Length];
        jointAngles = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            jointNames[i] = joints[i].name;
            //Need to get the right axis. For testing only
            jointAngles[i] = joints[i].localEulerAngles.z;
        }
    }

    private void OnEnable()
    {
        Debug.Log("OnEnable");
        rosConn.RegisterPublisher<PoseMsg>(topicTarget, 1000);
        rosConn.Subscribe<OvisJointsMsg>(topicJoints, ReceiveJointsCallback);

        rosConn.RegisterPublisher<OvisJointsMsg>(topicSim, 1000);

        lastUpdate = Time.realtimeSinceStartup;
    }

    private void Update()
    {
        if ((Time.realtimeSinceStartup - lastUpdate) >= updatePerSec)
        {
            Debug.Log("Update");
            var poseMsg = new PoseMsg
            {
                position = target.position.To<FLU>(),
                orientation = target.localRotation.To<FLU>()
            };

            rosConn.Publish(topicTarget, poseMsg);

            for (int i = 0; i < joints.Length; i++)
            {
                jointAngles[i] = joints[i].localEulerAngles.z;
            }

            var jointMsg = new OvisJointsMsg
            {
                joint_names = jointNames,
                joint_angles = jointAngles
            };

            rosConn.Publish(topicSim, jointMsg);

            lastUpdate = Time.realtimeSinceStartup;
        }
    }

    private void OnDisable()
    {
        Debug.Log("OnDisable");
        rosConn.Disconnect();
    }

    private void ReceiveJointsCallback(OvisJointsMsg msg)
    {
        Vector3 euler = joints[0].localEulerAngles;
        //Need to get the right axis. For testing only
        euler.y = msg.joint_angles[0];
        joints[0].localEulerAngles = euler;
    }
}
