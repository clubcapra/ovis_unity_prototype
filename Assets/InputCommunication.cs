using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;
using RosMessageTypes.Ovis;

public class InputCommunication : MonoBehaviour
{

    private InputMaster inputMaster;
    public OvisController ovisController;
    private InputAction movementInput;
    public XRRayInteractor rayCast;

    public int jointIndex = 0;
    public float speed = 500;

    public Material selectedMaterial;

    private Material lastMaterial;

    private float[] angles;

    // Start is called before the first frame update
    void Start()
    {
        angles = new float[ovisController.joints.Length];

        rayCast = GameObject.FindWithTag("RightHandController")?.GetComponent<XRRayInteractor>();
    }

    private void Awake()
    {
        inputMaster = new InputMaster();

        lastMaterial = ovisController.joints[jointIndex].visual.material;
        ovisController.joints[jointIndex].visual.material = selectedMaterial;
    }

    private void OnEnable()
    {
        inputMaster.Ovis.NextJoint.performed += CycleNextJoint;
        inputMaster.Ovis.NextJoint.Enable();
        inputMaster.Ovis.PreviousJoint.performed += CyclePreviousJoint;
        inputMaster.Ovis.PreviousJoint.Enable();
        inputMaster.Ovis.SelectJoint.performed += SelectJoint;
        inputMaster.Ovis.SelectJoint.Enable();
        movementInput = inputMaster.Ovis.Movement;
        movementInput.Enable();
    }

    void Update()
    {
        Vector2 moveDirection = movementInput.ReadValue<Vector2>();

        if (!ovisController.enabled)
        {
            angles[jointIndex] += speed * moveDirection.y * Time.deltaTime;

            ovisController.joints[jointIndex].SetAngularPosition(angles[jointIndex]);
        }
        else
        {
            float movement = speed * moveDirection.y * Time.deltaTime;

            var msg = new OvisJointGoalMsg
            {
                joint_index = (byte)jointIndex,
                joint_angle = ovisController.joints[jointIndex].GetAngularPosition() + movement
            };

            Debug.Log($"{msg.joint_index}, {msg.joint_angle}");

            ovisController.SendJointGoal(msg);
        }
    }

    private void CycleNextJoint(InputAction.CallbackContext obj)
    {
        if (jointIndex + 1 < ovisController.joints.Length)
        {
            SwitchJoint(jointIndex + 1);
        }
    }
    private void CyclePreviousJoint(InputAction.CallbackContext obj)
    {
        if (jointIndex - 1 >= 0)
        {
            SwitchJoint(jointIndex - 1);
        }
    }

    private void SwitchJoint(int newJointIndex)
    {
        ovisController.joints[jointIndex].visual.material = lastMaterial;

        lastMaterial = ovisController.joints[newJointIndex].visual.material;
        ovisController.joints[newJointIndex].visual.material = selectedMaterial;
        jointIndex = newJointIndex;
    }

    private void SelectJoint(InputAction.CallbackContext obj)
    {
        rayCast.TryGetCurrent3DRaycastHit(out RaycastHit hit);
        
        Debug.Log("this is the selected joint : " + hit.collider.name);

        switch (hit.collider.name)
        {
            case "ovis_base_0":
                SwitchJoint((int)moveableJoints.ovisBase -2);
                break;
            case "ovis_shoulder_0":
                SwitchJoint((int)moveableJoints.shoulder - 2);
                break;
            case "ovis_upper_arm_0":
                SwitchJoint((int)moveableJoints.upperArm - 2);
                break;
            case "ovis_elbow_0":
                SwitchJoint((int)moveableJoints.elbow - 2);
                break;
            case "ovis_forearm_0":
                SwitchJoint((int)moveableJoints.foreArm - 2);
                break;
            case "ovis_wrist_0":
                SwitchJoint((int)moveableJoints.wrist - 2);
                break;
            case "ovis_flange_0":
                SwitchJoint((int)moveableJoints.flange - 2);
                break;
            default:
                break;
        }
    }


    private enum moveableJoints
    {
        ovisBase = 1,
        shoulder = 2,
        upperArm = 3,
        elbow = 4,
        foreArm = 5,
        wrist = 6,
        flange = 7,
        leftPhalanx = 13,
        rightPhalanx = 18,
        leftFinger = 15,
        rightFinger = 20
    }
}
