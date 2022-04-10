using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointController : MonoBehaviour
{
    public enum JointAxis { X, Y, Z }

    public JointAxis axis;
    public Vector3 homeEulersOffset;
    public MeshRenderer visual;

    public float GetAngularPosition()
    {
        switch (axis)
        {
            case JointAxis.X: 
                return transform.localEulerAngles.x - homeEulersOffset.x;
            case JointAxis.Y: 
                return transform.localEulerAngles.y - homeEulersOffset.y;
            case JointAxis.Z: 
                return transform.localEulerAngles.z - homeEulersOffset.z;
        }

        return 0;
    }
    public void SetAngularPosition(float angular)
    {
        Vector3 eulerAngles = homeEulersOffset;

        switch (axis)
        {
            case JointAxis.X:
                eulerAngles.x += angular;
                break;
            case JointAxis.Y:
                eulerAngles.y += angular;
                break;
            case JointAxis.Z:
                eulerAngles.z += angular;
                break;
        }

        transform.localEulerAngles = eulerAngles;
    }

    public void SetHomePositionOffset(float homeOffset)
    {
        switch (axis)
        {
            case JointAxis.X:
                homeEulersOffset.x -= homeOffset;
                break;
            case JointAxis.Y:
                homeEulersOffset.y -= homeOffset;
                break;
            case JointAxis.Z:
                homeEulersOffset.z -= homeOffset;
                break;
        }        
    }

}
