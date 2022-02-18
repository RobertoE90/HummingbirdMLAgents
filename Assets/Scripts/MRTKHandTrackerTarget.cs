using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit;

public class MRTKHandTrackerTarget : MonoBehaviour, IMixedRealityHandJointHandler
{
    [SerializeField] private Transform _hummingbirdTarget;
    private Handedness _controlHandness;

    private void Awake()
    {
        CoreServices.InputSystem?.RegisterHandler<IMixedRealityHandJointHandler>(this);
        _controlHandness = Handedness.None;
    }

    public void OnHandJointsUpdated(InputEventData<IDictionary<TrackedHandJoint, MixedRealityPose>> eventData)
    {
        MixedRealityPose palmPose;
        if (!eventData.InputData.TryGetValue(TrackedHandJoint.Palm, out palmPose))
            return;

        var palmOrientation = Vector3.Dot(palmPose.Rotation * Vector3.down, Vector3.up);
        
        if (_controlHandness == Handedness.None) //search for hand
        {
            Debug.DrawRay(palmPose.Position, palmPose.Rotation * Vector3.down, Color.yellow);
            if(palmOrientation > 0.35f)
                _controlHandness = eventData.Handedness;
            
        }
        else
        {
            if (palmOrientation < 0)
                _controlHandness = Handedness.None;
        }

        if(eventData.Handedness == _controlHandness)
        {
            _hummingbirdTarget.position = palmPose.Position + palmPose.Rotation * Vector3.down * 0.075f;
            _hummingbirdTarget.rotation = palmPose.Rotation * Quaternion.Euler(0, 180, 180);
        }
    }
}