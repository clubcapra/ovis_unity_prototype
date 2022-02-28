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
    public string topicJoints = "/ovis_joints";
    public string topicTarget = "/ovis_target";

    public Transform target;
    public Transform[] joints;

    private ROSConnection ROSConn;

    // Start is called before the first frame update
    void Start()
    {
        ROSConn = GetComponent<ROSConnection>();
        ROSConn.Subscribe<OvisJointsMsg>(topicJoints, ReceiveJointsCallback);
        ROSConn.RegisterPublisher<PoseMsg>(topicTarget, 1000);
        StartCoroutine(SendTarget());
    }

    private void ReceiveJointsCallback(OvisJointsMsg msg)
    {
        Vector3 euler = joints[0].localEulerAngles;
        euler.z = (float)msg.eulers[0].z;
        joints[0].localEulerAngles = euler;
        //Debug.Log(msg.eulers[0].z);
    }

    IEnumerator SendTarget()
    {
        while (true)
        {
            var pose = new PoseMsg
            {
                position = target.position.To<FLU>(),
                orientation = target.localRotation.To<FLU>()
            };      

            ROSConn.Publish(topicTarget, pose);

            yield return new WaitForSeconds(1f);
        }
    }
}
