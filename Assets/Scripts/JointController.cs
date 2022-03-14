using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointController : MonoBehaviour
{
    public enum JointAxis { X, Y, Z }

    public JointAxis axis;
    public float offset;

    public float GetAngularPosition()
    {
        switch (axis)
        {
            case JointAxis.X: 
                return transform.localRotation.x - offset;
            case JointAxis.Y: 
                return transform.localRotation.y - offset;
            case JointAxis.Z: 
                return transform.localRotation.z - offset;
        }

        return 0;
    }
    public void SetAngularPosition(float angular)
    {
        Vector3 localRot = transform.localRotation.eulerAngles;

        switch (axis)
        {
            case JointAxis.X:
                localRot.x = angular + offset;
                break;
            case JointAxis.Y:
                localRot.y = angular + offset;
                break;
            case JointAxis.Z:
                localRot.z = angular + offset;
                break;
        }

        transform.localRotation = Quaternion.Euler(localRot);
    }

    public void SetHomePositionOffset(float homeOffset)
    {
        offset -= homeOffset;
    }

}
