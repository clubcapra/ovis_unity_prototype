// GENERATED AUTOMATICALLY FROM 'Assets/InputActions/InputMaster.inputactions'

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Utilities;

public class @InputMaster : IInputActionCollection, IDisposable
{
    public InputActionAsset asset { get; }
    public @InputMaster()
    {
        asset = InputActionAsset.FromJson(@"{
    ""name"": ""InputMaster"",
    ""maps"": [
        {
            ""name"": ""Ovis"",
            ""id"": ""04a2b468-17d0-42fa-9f14-a24548982bb5"",
            ""actions"": [
                {
                    ""name"": ""Next Joint"",
                    ""type"": ""Button"",
                    ""id"": ""d99e432e-6a6b-4b51-a569-b35a4caf1fcb"",
                    ""expectedControlType"": ""Button"",
                    ""processors"": """",
                    ""interactions"": """"
                },
                {
                    ""name"": ""Movement"",
                    ""type"": ""PassThrough"",
                    ""id"": ""fd4dad72-e9a8-4af2-a6fb-26184789d885"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """"
                },
                {
                    ""name"": ""Previous Joint"",
                    ""type"": ""Button"",
                    ""id"": ""453f529e-e1ba-49d3-b494-716ce1d491f7"",
                    ""expectedControlType"": ""Button"",
                    ""processors"": """",
                    ""interactions"": """"
                }
            ],
            ""bindings"": [
                {
                    ""name"": """",
                    ""id"": ""49bf7b4b-00a5-4d90-8b3e-8aa3cddf1403"",
                    ""path"": ""<Gamepad>/rightShoulder"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Next Joint"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""D-Pad"",
                    ""id"": ""e3c27e07-3292-46d7-9353-63bc979d781c"",
                    ""path"": ""2DVector"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""7ef25f53-f1f2-4fbf-b2eb-bb709c32258e"",
                    ""path"": ""<Gamepad>/dpad"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""7ea020f3-38b2-43bc-a2fd-e24326f21bba"",
                    ""path"": ""<Gamepad>/dpad"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""058cfeeb-f8a7-4b2a-acc7-f4c0a9abeb4a"",
                    ""path"": ""<Gamepad>/dpad"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""21fcd863-2ae1-4ceb-8e11-8610fbef8272"",
                    ""path"": ""<Gamepad>/dpad"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""WASD"",
                    ""id"": ""86ac4a3d-afa2-40d4-a91e-c643240b7f54"",
                    ""path"": ""2DVector"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""fbcba083-2510-445c-b2b8-98b6386e2d51"",
                    ""path"": ""<Keyboard>/w"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""5324d190-0006-485f-a9f0-cf6ce825603e"",
                    ""path"": ""<Keyboard>/s"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""a8486d4e-1539-4f0f-a705-c07d74686168"",
                    ""path"": ""<Keyboard>/a"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""805a434e-6e78-4b35-aa56-dfc9fa983748"",
                    ""path"": ""<Keyboard>/d"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""2D Vector"",
                    ""id"": ""99f791cf-8714-4f10-9c70-1d2c9ea3a6f9"",
                    ""path"": ""2DVector"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""195b7fd3-a621-4dca-80d6-a143f9d09f83"",
                    ""path"": ""<Gamepad>/leftStick/up"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""b164e38a-5c52-410b-ab64-37a597ee64ab"",
                    ""path"": ""<Gamepad>/leftStick/down"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""c236838b-b0c4-4260-97b1-56b1f32aac76"",
                    ""path"": ""<Gamepad>/leftStick/left"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""e3341b80-273c-4dd7-afb4-7c7da7758110"",
                    ""path"": ""<Gamepad>/leftStick/right"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": """",
                    ""id"": ""aaa55879-d7dd-4ab6-88c6-851c1deb9f60"",
                    ""path"": ""<Gamepad>/leftShoulder"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Previous Joint"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                }
            ]
        },
        {
            ""name"": ""Camera"",
            ""id"": ""64570259-5818-4cf8-bcc8-e612c9a5955c"",
            ""actions"": [
                {
                    ""name"": ""Move Camera"",
                    ""type"": ""PassThrough"",
                    ""id"": ""8c53483e-323d-4c59-9098-447a48428b3e"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """"
                },
                {
                    ""name"": ""Turn Camera"",
                    ""type"": ""PassThrough"",
                    ""id"": ""81b9766a-694e-4d3d-b2ca-946aa524cc00"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """"
                }
            ],
            ""bindings"": [
                {
                    ""name"": ""2D Vector"",
                    ""id"": ""f8b4a577-517d-4da6-a0fa-a60c2df2251a"",
                    ""path"": ""2DVector"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Move Camera"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""ee942b01-d4c2-4d32-b896-4c59ffdff58f"",
                    ""path"": ""<Gamepad>/leftStick/up"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Move Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""9823893a-4822-48b8-9ef6-27ace1032732"",
                    ""path"": ""<Gamepad>/leftStick/down"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Move Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""9ca0f710-f90f-447b-9c97-144e694a7411"",
                    ""path"": ""<Gamepad>/leftStick/left"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Move Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""cbc09a36-9103-4443-99b8-e087b47d462c"",
                    ""path"": ""<Gamepad>/leftStick/right"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Move Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""2D Vector"",
                    ""id"": ""18a1ec0b-474d-46cb-96dd-79b1efec98c0"",
                    ""path"": ""2DVector"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Turn Camera"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""15dde303-f923-442f-942f-9fdd03c5926d"",
                    ""path"": ""<Gamepad>/rightStick/up"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Turn Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""cb216398-64c5-4b7d-add0-a01cda59eb35"",
                    ""path"": ""<Gamepad>/rightStick/down"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Turn Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""d8e76b44-2c41-47e4-91d7-99ded4eb181d"",
                    ""path"": ""<Gamepad>/rightStick/left"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Turn Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""b1847a4f-ad33-4730-90f1-74d626105ea2"",
                    ""path"": ""<Gamepad>/rightStick/right"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Xbox Control Scheme"",
                    ""action"": ""Turn Camera"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                }
            ]
        }
    ],
    ""controlSchemes"": [
        {
            ""name"": ""Xbox Control Scheme"",
            ""bindingGroup"": ""Xbox Control Scheme"",
            ""devices"": [
                {
                    ""devicePath"": ""<XInputController>"",
                    ""isOptional"": false,
                    ""isOR"": false
                }
            ]
        }
    ]
}");
        // Ovis
        m_Ovis = asset.FindActionMap("Ovis", throwIfNotFound: true);
        m_Ovis_NextJoint = m_Ovis.FindAction("Next Joint", throwIfNotFound: true);
        m_Ovis_Movement = m_Ovis.FindAction("Movement", throwIfNotFound: true);
        m_Ovis_PreviousJoint = m_Ovis.FindAction("Previous Joint", throwIfNotFound: true);
        // Camera
        m_Camera = asset.FindActionMap("Camera", throwIfNotFound: true);
        m_Camera_MoveCamera = m_Camera.FindAction("Move Camera", throwIfNotFound: true);
        m_Camera_TurnCamera = m_Camera.FindAction("Turn Camera", throwIfNotFound: true);
    }

    public void Dispose()
    {
        UnityEngine.Object.Destroy(asset);
    }

    public InputBinding? bindingMask
    {
        get => asset.bindingMask;
        set => asset.bindingMask = value;
    }

    public ReadOnlyArray<InputDevice>? devices
    {
        get => asset.devices;
        set => asset.devices = value;
    }

    public ReadOnlyArray<InputControlScheme> controlSchemes => asset.controlSchemes;

    public bool Contains(InputAction action)
    {
        return asset.Contains(action);
    }

    public IEnumerator<InputAction> GetEnumerator()
    {
        return asset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public void Enable()
    {
        asset.Enable();
    }

    public void Disable()
    {
        asset.Disable();
    }

    // Ovis
    private readonly InputActionMap m_Ovis;
    private IOvisActions m_OvisActionsCallbackInterface;
    private readonly InputAction m_Ovis_NextJoint;
    private readonly InputAction m_Ovis_Movement;
    private readonly InputAction m_Ovis_PreviousJoint;
    public struct OvisActions
    {
        private @InputMaster m_Wrapper;
        public OvisActions(@InputMaster wrapper) { m_Wrapper = wrapper; }
        public InputAction @NextJoint => m_Wrapper.m_Ovis_NextJoint;
        public InputAction @Movement => m_Wrapper.m_Ovis_Movement;
        public InputAction @PreviousJoint => m_Wrapper.m_Ovis_PreviousJoint;
        public InputActionMap Get() { return m_Wrapper.m_Ovis; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(OvisActions set) { return set.Get(); }
        public void SetCallbacks(IOvisActions instance)
        {
            if (m_Wrapper.m_OvisActionsCallbackInterface != null)
            {
                @NextJoint.started -= m_Wrapper.m_OvisActionsCallbackInterface.OnNextJoint;
                @NextJoint.performed -= m_Wrapper.m_OvisActionsCallbackInterface.OnNextJoint;
                @NextJoint.canceled -= m_Wrapper.m_OvisActionsCallbackInterface.OnNextJoint;
                @Movement.started -= m_Wrapper.m_OvisActionsCallbackInterface.OnMovement;
                @Movement.performed -= m_Wrapper.m_OvisActionsCallbackInterface.OnMovement;
                @Movement.canceled -= m_Wrapper.m_OvisActionsCallbackInterface.OnMovement;
                @PreviousJoint.started -= m_Wrapper.m_OvisActionsCallbackInterface.OnPreviousJoint;
                @PreviousJoint.performed -= m_Wrapper.m_OvisActionsCallbackInterface.OnPreviousJoint;
                @PreviousJoint.canceled -= m_Wrapper.m_OvisActionsCallbackInterface.OnPreviousJoint;
            }
            m_Wrapper.m_OvisActionsCallbackInterface = instance;
            if (instance != null)
            {
                @NextJoint.started += instance.OnNextJoint;
                @NextJoint.performed += instance.OnNextJoint;
                @NextJoint.canceled += instance.OnNextJoint;
                @Movement.started += instance.OnMovement;
                @Movement.performed += instance.OnMovement;
                @Movement.canceled += instance.OnMovement;
                @PreviousJoint.started += instance.OnPreviousJoint;
                @PreviousJoint.performed += instance.OnPreviousJoint;
                @PreviousJoint.canceled += instance.OnPreviousJoint;
            }
        }
    }
    public OvisActions @Ovis => new OvisActions(this);

    // Camera
    private readonly InputActionMap m_Camera;
    private ICameraActions m_CameraActionsCallbackInterface;
    private readonly InputAction m_Camera_MoveCamera;
    private readonly InputAction m_Camera_TurnCamera;
    public struct CameraActions
    {
        private @InputMaster m_Wrapper;
        public CameraActions(@InputMaster wrapper) { m_Wrapper = wrapper; }
        public InputAction @MoveCamera => m_Wrapper.m_Camera_MoveCamera;
        public InputAction @TurnCamera => m_Wrapper.m_Camera_TurnCamera;
        public InputActionMap Get() { return m_Wrapper.m_Camera; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(CameraActions set) { return set.Get(); }
        public void SetCallbacks(ICameraActions instance)
        {
            if (m_Wrapper.m_CameraActionsCallbackInterface != null)
            {
                @MoveCamera.started -= m_Wrapper.m_CameraActionsCallbackInterface.OnMoveCamera;
                @MoveCamera.performed -= m_Wrapper.m_CameraActionsCallbackInterface.OnMoveCamera;
                @MoveCamera.canceled -= m_Wrapper.m_CameraActionsCallbackInterface.OnMoveCamera;
                @TurnCamera.started -= m_Wrapper.m_CameraActionsCallbackInterface.OnTurnCamera;
                @TurnCamera.performed -= m_Wrapper.m_CameraActionsCallbackInterface.OnTurnCamera;
                @TurnCamera.canceled -= m_Wrapper.m_CameraActionsCallbackInterface.OnTurnCamera;
            }
            m_Wrapper.m_CameraActionsCallbackInterface = instance;
            if (instance != null)
            {
                @MoveCamera.started += instance.OnMoveCamera;
                @MoveCamera.performed += instance.OnMoveCamera;
                @MoveCamera.canceled += instance.OnMoveCamera;
                @TurnCamera.started += instance.OnTurnCamera;
                @TurnCamera.performed += instance.OnTurnCamera;
                @TurnCamera.canceled += instance.OnTurnCamera;
            }
        }
    }
    public CameraActions @Camera => new CameraActions(this);
    private int m_XboxControlSchemeSchemeIndex = -1;
    public InputControlScheme XboxControlSchemeScheme
    {
        get
        {
            if (m_XboxControlSchemeSchemeIndex == -1) m_XboxControlSchemeSchemeIndex = asset.FindControlSchemeIndex("Xbox Control Scheme");
            return asset.controlSchemes[m_XboxControlSchemeSchemeIndex];
        }
    }
    public interface IOvisActions
    {
        void OnNextJoint(InputAction.CallbackContext context);
        void OnMovement(InputAction.CallbackContext context);
        void OnPreviousJoint(InputAction.CallbackContext context);
    }
    public interface ICameraActions
    {
        void OnMoveCamera(InputAction.CallbackContext context);
        void OnTurnCamera(InputAction.CallbackContext context);
    }
}