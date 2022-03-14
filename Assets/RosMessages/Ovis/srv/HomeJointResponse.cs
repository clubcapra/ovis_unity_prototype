//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ovis
{
    [Serializable]
    public class HomeJointResponse : Message
    {
        public const string k_RosMessageName = "ovis_msgs/HomeJoint";
        public override string RosMessageName => k_RosMessageName;

        public float[] home_joint_positions;

        public HomeJointResponse()
        {
            this.home_joint_positions = new float[0];
        }

        public HomeJointResponse(float[] home_joint_positions)
        {
            this.home_joint_positions = home_joint_positions;
        }

        public static HomeJointResponse Deserialize(MessageDeserializer deserializer) => new HomeJointResponse(deserializer);

        private HomeJointResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.home_joint_positions, sizeof(float), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.home_joint_positions);
            serializer.Write(this.home_joint_positions);
        }

        public override string ToString()
        {
            return "HomeJointResponse: " +
            "\nhome_joint_positions: " + System.String.Join(", ", home_joint_positions.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
