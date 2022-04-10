using RosMessageTypes.Ovis;
using RosMessageTypes.Std;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(OvisController))]
public class JointGoalController : MonoBehaviour
{
    public int jointIndex = 0;
    public float speed = 500;

    public Material selectedMaterial;

    private Material lastMaterial;
    private OvisController ovisController;
    private Keyboard keyboard;

    private void Awake()
    {
        keyboard = Keyboard.current;

        ovisController = GetComponent<OvisController>();

        ovisController.OnHomeReceived += () => 
        {
            lastMaterial = ovisController.joints[jointIndex].visual.material;
            ovisController.joints[jointIndex].visual.material = selectedMaterial;
            enabled = true; 
        };
    }

    private void SwitchJoint(int newJointIndex)
    {
        ovisController.joints[jointIndex].visual.material = lastMaterial;

        lastMaterial = ovisController.joints[newJointIndex].visual.material;
        ovisController.joints[newJointIndex].visual.material = selectedMaterial;
        jointIndex = newJointIndex;
    }

    // Update is called once per frame
    void Update()
    {
        int direction = 0;
        if (keyboard[Key.UpArrow].isPressed)
            direction = 1;
        else if (keyboard[Key.DownArrow].isPressed)
            direction = -1;

        if (keyboard[Key.E].wasPressedThisFrame && jointIndex + 1 < ovisController.joints.Length)
        {
            SwitchJoint(jointIndex + 1);
        }
        else if (keyboard[Key.Q].wasPressedThisFrame && jointIndex - 1 >= 0)
        {
            SwitchJoint(jointIndex - 1);
        }

        if (direction != 0)
        {
            float movement = speed * direction * Time.deltaTime;

            var msg = new OvisJointGoalMsg
            {
                joint_index = (byte)jointIndex,
                joint_angle = ovisController.joints[jointIndex].GetAngularPosition() + movement
            };

            Debug.Log($"{msg.joint_index}, {msg.joint_angle}");

            ovisController.SendJointGoal(msg);
        }
    }
}
