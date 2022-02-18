using System;
using System.Collections;
using UnityEngine;

public class HeadNeckController : MonoBehaviour
{
    [SerializeField] private bool _debug;
    [Space(20)]
    [SerializeField] [Range(-1, 1)] private float _horizontalControl;
    [SerializeField] [Range(-1, 1)] private float _verticalControl;
    [SerializeField] [Range(-1, 1)] private float _constrainHeadPosition;
    private float _currentHorizontalControl;
    private float _currentVerticalControl;

    [Space(10)]
    [SerializeField] private Vector3 _headLockCenter;
    [SerializeField] private Vector3 _headLockRadius;

    [Space(10)]
    [SerializeField] private float _horizontalMaxApertureAngle;

    [Header("References")]
    [SerializeField] private Transform _root;
    [SerializeField] private Transform _neck;
    [SerializeField] private Transform _head;
    [SerializeField] private Transform _beackTip;

    private HeadPose _targetHeadPose;
    private Quaternion _neckInitialRotation;

    private float _nextHeadMoveAvailableTime = float.MaxValue;


    public void Initialize()
    {
        _neckInitialRotation = _neck.localRotation;
        _nextHeadMoveAvailableTime = Time.time + 2f;
    }

    /// <summary>
    /// Immediate Set the parameters for head neck controller and repositionates the head and neck
    /// </summary>
    /// <param name="horizontalControl"> horizontal rotation from -1 to 1 to be converted to 0 360 in world rotation</param>
    /// <param name="verticalControl"> vertical rotation from -1 to 1 to be converted to -90 90 in world rotation</param>
    /// <param name="constrainHeadFloatValue"> if is greater than 0 the head position will be constrained </param>
    public void ImmediateSetHeatPose(float horizontalControl, float verticalControl, float constrainHeadFloatValue)
    {
        SetTargetHeadPoseParams(horizontalControl, verticalControl, constrainHeadFloatValue, true);
        ComputeTargetHeadPose();
        ApplyConstrainHeadPose();
    }

    /// <summary>
    /// Set the parameters for head neck controller
    /// </summary>
    /// <param name="horizontalControl"> horizontal rotation from -1 to 1 to be converted to 0 360 in world rotation</param>
    /// <param name="verticalControl"> vertical rotation from -1 to 1 to be converted to -90 90 in world rotation</param>
    /// <param name="constrainHeadFloatValue"> if is greater than 0 the head position will be constrained </param>
    public void SetTargetHeadPoseParams(float horizontalControl, float verticalControl, float constrainHeadFloatValue, bool forceApply = false)
    {
        var angleBetweenCalls = Vector2.Angle(new Vector2(horizontalControl, verticalControl), new Vector2(_currentHorizontalControl, _currentVerticalControl));

        if ((constrainHeadFloatValue > 0f && Time.time >= _nextHeadMoveAvailableTime && angleBetweenCalls > 30f) || forceApply)
        //if (constrainHeadFloatValue > 0f || forceApply)
        {
            //_nextHeadMoveAvailableTime += UnityEngine.Random.Range(0.5f, 2.5f);
            _nextHeadMoveAvailableTime += 0.15f;

            _horizontalControl = horizontalControl;
            _verticalControl = verticalControl;
        }

        //updates the head pose
        _constrainHeadPosition = constrainHeadFloatValue;
        _currentHorizontalControl = Mathf.Lerp(_currentHorizontalControl, _horizontalControl, 10 * Time.deltaTime);
        _currentVerticalControl = Mathf.Lerp(_currentVerticalControl, _verticalControl, 10 * Time.deltaTime);
        ComputeTargetHeadPose();
        ApplyConstrainHeadPose();
    }

    private void ComputeTargetHeadPose()
    {
        var normalizedHorizontal = _currentHorizontalControl * 0.5f + 0.5f;

        var targetRotation = Quaternion.Euler(0, normalizedHorizontal * 360, 0) * Quaternion.Euler(_currentVerticalControl * 85f, 0, 0);

        _targetHeadPose.LookAtPosition =
            _neck.position +
            targetRotation * Vector3.back * 2f;

        var nextContrainPositionValue = _constrainHeadPosition > 0;
        if (!_targetHeadPose.ContrainPosition && nextContrainPositionValue)
            _targetHeadPose.TargetHeadPosition = _head.position;
        
        _targetHeadPose.ContrainPosition = nextContrainPositionValue;
    }

    private void ApplyConstrainHeadPose()
    {
        var cposition = _head.position;
        if (_constrainHeadPosition > 0)
            cposition = ConstrainHeadPosition();

        var crotation = ConstrainHeadRotation();

        var neckHeadDelta = cposition - _neck.position;
        var targetNeckRotation = Quaternion.LookRotation(neckHeadDelta, _neck.parent.rotation * _neckInitialRotation * Vector3.down) * Quaternion.Euler(180, 0, 0);
        _neck.rotation = targetNeckRotation;
        _head.position = cposition;
        _head.rotation = crotation;

    }

    private Vector3 ConstrainHeadPosition()
    {        
        var localSpaceHeadPosition = _root.InverseTransformPoint(_targetHeadPose.TargetHeadPosition);
        
        var scaleMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, _headLockRadius);
        var delta = localSpaceHeadPosition - _headLockCenter;
        delta = scaleMatrix.inverse.MultiplyPoint3x4(delta);
        var deltaDistance = delta.magnitude;
        if (deltaDistance > 1)
            delta *= 1 / deltaDistance;
        delta = scaleMatrix.MultiplyPoint3x4(delta);

        return _root.TransformPoint(_headLockCenter + delta);
    }

    private Quaternion ConstrainHeadRotation()
    {
        //var toTargetRotation = Quaternion.LookRotation(_targetHeadPose.LookAtPosition - _head.position, (Vector3.Dot(_root.up, Vector3.up) > 0f) ? Vector3.up : Vector3.down);
        var toTargetRotation = Quaternion.LookRotation(_targetHeadPose.LookAtPosition - _head.position,  Vector3.up);

        var transformMatrix = Matrix4x4.TRS(Vector3.zero, _neckInitialRotation * _neck.parent.rotation, Vector3.one);
        var headCandidateForwardLocal = transformMatrix.inverse.MultiplyPoint3x4(toTargetRotation * Vector3.forward * -1);
        headCandidateForwardLocal = ClampVector(headCandidateForwardLocal, _horizontalMaxApertureAngle);

        var globalUpBodyVector = new Vector3(0, _root.up.y, 0) * 10;
        if (_root.up.y == 0)
            globalUpBodyVector.y = 10;

        //magnitude of dir needs to be 1
        Vector3 ClampVector(Vector3 dir, float maxAngle)
        {
            var minZ = Mathf.Cos(Mathf.Deg2Rad * maxAngle);
            if (dir.z < minZ)
                dir.z = minZ;

            var maxSides = Mathf.Sin(Mathf.Deg2Rad * maxAngle);

            var sidesProjection = new Vector2(dir.x, dir.y);
            var projectionDistance = sidesProjection.magnitude;
            if (projectionDistance > maxSides)
                sidesProjection *= maxSides / projectionDistance;

            dir.x = sidesProjection.x;
            dir.y = sidesProjection.y;

            return dir;
        }

        return Quaternion.LookRotation(
            transformMatrix.MultiplyPoint3x4(headCandidateForwardLocal), 
            globalUpBodyVector);
    }

    private void OnDrawGizmos()
    {
        if (!_debug)
            return;

        Gizmos.color = Color.white;
        Gizmos.matrix = Matrix4x4.Translate(_root.position) * 
            Matrix4x4.Translate(_root.TransformDirection(_headLockCenter)) * 
            Matrix4x4.Rotate(_root.rotation) *
            Matrix4x4.Scale(_headLockRadius);
        Gizmos.DrawWireSphere(Vector3.zero, 1);

        Gizmos.matrix = Matrix4x4.identity;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(_head.position, _targetHeadPose.LookAtPosition);
        Gizmos.DrawWireCube(_targetHeadPose.LookAtPosition, Vector3.one * 0.05f);
        if (_targetHeadPose.ContrainPosition)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(_targetHeadPose.TargetHeadPosition, 0.025f);
        }
    }

    private struct HeadPose
    {
        public Vector3 LookAtPosition;
        public Vector3 TargetHeadPosition;
        public bool ContrainPosition;
    }
}
