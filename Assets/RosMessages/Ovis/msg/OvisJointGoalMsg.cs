//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ovis
{
    [Serializable]
    public class OvisJointGoalMsg : Message
    {
        public const string k_RosMessageName = "ovis_msgs/OvisJointGoal";
        public override string RosMessageName => k_RosMessageName;

        public byte joint_index;
        public float joint_angle;

        public OvisJointGoalMsg()
        {
            this.joint_index = 0;
            this.joint_angle = 0.0f;
        }

        public OvisJointGoalMsg(byte joint_index, float joint_angle)
        {
            this.joint_index = joint_index;
            this.joint_angle = joint_angle;
        }

        public static OvisJointGoalMsg Deserialize(MessageDeserializer deserializer) => new OvisJointGoalMsg(deserializer);

        private OvisJointGoalMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.joint_index);
            deserializer.Read(out this.joint_angle);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.joint_index);
            serializer.Write(this.joint_angle);
        }

        public override string ToString()
        {
            return "OvisJointGoalMsg: " +
            "\njoint_index: " + joint_index.ToString() +
            "\njoint_angle: " + joint_angle.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
