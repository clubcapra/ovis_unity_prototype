using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using UnityEngine.InputSystem;

public class InputController : Controller
{
    public readonly int jointCount = 20;
    private InputMaster inputMaster;
    private InputAction movement;
    private InputAction moveCamera;
    private InputAction turnCamera;

    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

    private ArticulationBody[] articulationChainIC;
    // Stores original colors of the part being highlighted
    private Color[] prevColorIC;
    private int previousIndexIC;

    //[InspectorReadOnly(hideInEditMode: true)]
    //public string selectedJoint;
    //[HideInInspector]
    //public int selectedIndex;

    /*public ControlType control = ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2*/

    private Gamepad gamepad;

    //[Tooltip("Color to highlight the currently selected join")]
    //public Color highLightColor = new Color(1.0f, 0, 0, 1.0f);

    private void Awake()
    {
        inputMaster = new InputMaster();
    }

    private void OnEnable()
    {
        inputMaster.Ovis.NextJoint.performed += CycleNextJoint;
        inputMaster.Ovis.NextJoint.Enable();
        inputMaster.Ovis.PreviousJoint.performed += CyclePreviousJoint;
        inputMaster.Ovis.PreviousJoint.Enable();

        movement = inputMaster.Ovis.Movement;
        movement.Enable();

        /*moveCamera = inputMaster.Camera.MoveCamera;
        moveCamera.Enable();
        turnCamera = inputMaster.Camera.TurnCamera;
        turnCamera.Enable();*/
    }

    private void OnDisable()
    {
        //inputMaster.Ovis.NextJoint.Disable();
    }

    // Start is called before the first frame update
    void Start()
    {
        previousIndexIC = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
        articulationChainIC = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChainIC)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        DisplaySelectedJoint(selectedIndex);
        StoreJointColors(selectedIndex);
    }

    // Update is called once per frame
    void Update()
    {
        SetSelectedJointIndex(selectedIndex); // to make sure it is in the valid range
        UpdateDirection(selectedIndex);
    }

    private void CycleNextJoint(InputAction.CallbackContext obj)
    {
        selectedIndex++;
        while (!Enum.IsDefined(typeof(moveableJoints), selectedIndex))
        {
            if (selectedIndex >= jointCount)
                selectedIndex = 1;
            else
                selectedIndex++;
        }
        SetSelectedJointIndex(selectedIndex);
        Highlight(selectedIndex);
    }

    private void CyclePreviousJoint(InputAction.CallbackContext obj)
    {
        selectedIndex--;
        while (!Enum.IsDefined(typeof(moveableJoints), selectedIndex))
        {
            if (selectedIndex < 0)
                selectedIndex = 30;
            selectedIndex--;
        }
        SetSelectedJointIndex(selectedIndex);
        Highlight(selectedIndex);
    }

    void SetSelectedJointIndex(int index)
    {
        if (articulationChainIC.Length > 0)
        {
            selectedIndex = (index + articulationChainIC.Length) % articulationChainIC.Length;
        }
    }

    /// <summary>
    /// Highlights the color of the robot by changing the color of the part to a color set by the user in the inspector window
    /// </summary>
    /// <param name="selectedIndex">Index of the link selected in the Articulation Chain</param>
    private void Highlight(int selectedIndex)
    {
        if (selectedIndex == previousIndexIC || selectedIndex < 0 || selectedIndex >= articulationChainIC.Length)
        {
            return;
        }

        // reset colors for the previously selected joint
        ResetJointColors(previousIndexIC);

        // store colors for the current selected joint
        StoreJointColors(selectedIndex);

        DisplaySelectedJoint(selectedIndex);
        Renderer[] rendererList = articulationChainIC[selectedIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();

        // set the color of the selected join meshes to the highlight color
        foreach (var mesh in rendererList)
        {
            MaterialExtensions.SetMaterialColor(mesh.material, highLightColor);
        }
    }

    void DisplaySelectedJoint(int selectedIndex)
    {
        if (selectedIndex < 0 || selectedIndex >= articulationChainIC.Length)
        {
            return;
        }
        selectedJoint = articulationChainIC[selectedIndex].name + " (" + selectedIndex + ")";
    }

    /// <summary>
    /// Sets the direction of movement of the joint on every update
    /// </summary>
    /// <param name="jointIndex">Index of the link selected in the Articulation Chain</param>
    private void UpdateDirection(int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= articulationChainIC.Length)
        {
            return;
        }

        Vector2 moveDirection = movement.ReadValue<Vector2>();
        JointControl current = articulationChainIC[jointIndex].GetComponent<JointControl>();
        if (previousIndexIC != jointIndex)
        {
            JointControl previous = articulationChainIC[previousIndexIC].GetComponent<JointControl>();
            previous.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
            previousIndexIC = jointIndex;
        }

        if (current.controltype != control)
        {
            UpdateControlType(current);
        }

        // The moveable joint have the following indexes :
        // 2, 3, 4, 5, 6, 7, (13 + 18), (15 + 20)
        // 13 + 18 & 15 + 20 are the fingers of the hand and should together 
        switch (selectedIndex) 
        {
            case 2:
                if (moveDirection.x > 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                else if (moveDirection.x < 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                else
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

            case 3:
                if (moveDirection.y > 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                else if (moveDirection.y < 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                else
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

            case 4:
                if (moveDirection.y > 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                else if (moveDirection.y < 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                else
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

            // Elbow wrist?
            case 5:
                if (moveDirection.x > 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                else if (moveDirection.x < 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                else
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

            case 6:
                if (moveDirection.y > 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                else if (moveDirection.y < 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                else
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

            // Wrist
            case 7:
                if (moveDirection.x > 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                else if (moveDirection.x < 0)
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                else
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

            // 13 + 18 = Phalanx
            case 13:
                JointControl joint18 = articulationChainIC[18].GetComponent<JointControl>();
                if (moveDirection.x > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint18.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }  
                else if (moveDirection.x < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint18.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                // Opens and closes the hand
                else if (moveDirection.y > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint18.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                else if (moveDirection.y < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint18.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }
                else
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                    joint18.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                }  
                break;

            case 18:
                JointControl joint13 = articulationChainIC[13].GetComponent<JointControl>();
                if (moveDirection.x > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint13.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                } 
                else if (moveDirection.x < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint13.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                // Opens and closes the hand
                else if (moveDirection.y > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint13.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                else if (moveDirection.y < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint13.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }

                else
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                    joint13.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                }
                break;

            // 15 + 20 = Finger tip
            case 15:
                JointControl joint20 = articulationChainIC[20].GetComponent<JointControl>();
                if (moveDirection.x > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint20.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }

                else if (moveDirection.x < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint20.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                // Opens and closes the hand
                else if (moveDirection.y > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint20.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                else if (moveDirection.y < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint20.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }

                else
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                    joint20.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                }
                break;

            case 20:
                JointControl joint15 = articulationChainIC[15].GetComponent<JointControl>();
                // Translates the hand
                if (moveDirection.x > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint15.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }
                else if (moveDirection.x < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint15.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                // Opens and closes the hand
                else if (moveDirection.y > 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                    joint15.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                }
                else if (moveDirection.y < 0)
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Negative;
                    joint15.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.Positive;
                }
                else
                {
                    current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                    joint15.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                }   
                break;

            default:
                current.direction = (Unity.Robotics.UrdfImporter.Control.RotationDirection)RotationDirection.None;
                break;

        };
    }

    /// <summary>
    /// Stores original color of the part being highlighted
    /// </summary>
    /// <param name="index">Index of the part in the Articulation chain</param>
    private void StoreJointColors(int index)
    {
        Renderer[] materialLists = articulationChainIC[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        prevColorIC = new Color[materialLists.Length];
        for (int counter = 0; counter < materialLists.Length; counter++)
        {
            prevColorIC[counter] = MaterialExtensions.GetMaterialColor(materialLists[counter]);
        }
    }

    /// <summary>
    /// Resets original color of the part being highlighted
    /// </summary>
    /// <param name="index">Index of the part in the Articulation chain</param>
    private void ResetJointColors(int index)
    {
        Renderer[] previousRendererList = articulationChainIC[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        for (int counter = 0; counter < previousRendererList.Length; counter++)
        {
            MaterialExtensions.SetMaterialColor(previousRendererList[counter].material, prevColorIC[counter]);
        }
    }

    /*public void UpdateControlType(JointControl joint)
    {
        joint.controltype = (Unity.Robotics.UrdfImporter.Control.ControlType)control;
        if (control == ControlType.PositionControl)
        {
            ArticulationDrive drive = joint.joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }*/

    /*public void OnGUI()
    {
        GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
        centeredStyle.alignment = TextAnchor.UpperCenter;
        GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press Q / E keys to select a robot joint.", centeredStyle);
        GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 20), "Press WASD keys to move " + selectedJoint + ".", centeredStyle);
    }*/

    private enum moveableJoints 
    { 
        shoulder = 2, 
        Arm = 3, 
        elbow = 4, 
        foreArme = 5, 
        backWrits = 6, 
        frontWrist = 7, 
        leftPhalanx = 13, 
        rightPhalanx = 18, 
        leftFinger = 15,
        rightFinger = 20 
    }
}
