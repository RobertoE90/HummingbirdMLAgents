using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[CreateAssetMenu(fileName = "ChainPose", menuName = "Chain/ChainPose")]
public class ChainPose : ScriptableObject
{
    [SerializeField] private Vector3[] _eulerAngles;
    public int JoinCount => _eulerAngles.Length;

    public ChainPose()
    {

    }

    public void Initialize(Vector3[] eulerAngles)
    {
        _eulerAngles = eulerAngles;
    }


    public Vector3 GetEulerAngles(int index)
    {
        return _eulerAngles[index];
    }

    public Quaternion GetRotation(int index)
    {
        try {
            return Quaternion.Euler(_eulerAngles[index]);
        }
        catch(IndexOutOfRangeException iore)
        {
            return Quaternion.Euler(_eulerAngles[_eulerAngles.Length - 1]);
        }
        catch(Exception e)
        {
            return Quaternion.identity;
        }
    }
    
}
