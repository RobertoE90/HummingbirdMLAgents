using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChainTargetController : MonoBehaviour
{
    [SerializeField][Range(-1f, 1f)] private float _horizontalLerp;
    [SerializeField][Range(-1f, 1f)] private float _verticalLerp;
    [Space(10)]
    [SerializeField] private ChainToTarget _chainToTarget;
    [Space(20)]
    [SerializeField] private Transform[] _joins;
    private Vector3[] _initPositions;
    private Quaternion[] _initRotations;

    [SerializeField] private ChainPose _upPose;
    [SerializeField] private ChainPose _downPose;
    [SerializeField] private ChainPose _leftPose;
    [SerializeField] private ChainPose _rightPose;
    [Space(20)]
    [Header("Debugs")]
    [SerializeField] private bool _debugPoses;
    [SerializeField] private bool _debugTargetChain;
    
    private ChainPose _lerpedPose;
    private Vector3[] _chainTargetPositions;
    public Vector3[] TargetPositions => _chainTargetPositions;

    private Quaternion[] _chainTargetRotations;
    public Quaternion[] TargetRotations => _chainTargetRotations;

    private void Awake()
    {
        Initialize();
    }
    
    private void Update()
    {
        ForceUpdatePose();
    }

    private void Initialize()
    {
        _initPositions = new Vector3[_joins.Length];
        _initRotations = new Quaternion[_joins.Length];
        for (var i = 0; i < _joins.Length; i++)
        {
            _initPositions[i] = _joins[i].localPosition;
            _initRotations[i] = _joins[i].localRotation;
        }
    }

    public void SetChainControlsValues(float horizontal, float vertical)
    {
        _horizontalLerp = horizontal;
        _verticalLerp = vertical;
    }

    public void ForceUpdatePose()
    {
        if (_initPositions == null || _initRotations == null)
            Initialize();

        _lerpedPose = InterpolatePoses();
        TransformChainWithPose(_lerpedPose, out _chainTargetPositions, out _chainTargetRotations);
        _chainToTarget.FitChainToPositions(_chainTargetPositions);
    }

    private void OnDrawGizmos()
    {

        if (_debugTargetChain)
            DrawChain(_lerpedPose, new Color(0, 255, 255), 0.015f, false);

        Gizmos.matrix = Matrix4x4.identity;

        if (_debugPoses)
        {
            DrawChain(_upPose, new Color(75, 0, 0));
            DrawChain(_downPose, new Color(150, 0, 0));
            DrawChain(_leftPose, new Color(0, 75, 0));
            DrawChain(_rightPose, new Color(0, 150, 0));
        }

        void DrawChain(ChainPose chain, Color color, float radius = 0.01f, bool wire = true)
        {
            TransformChainWithPose(chain, out var positions, out var rotations);
            Gizmos.color = color;
            for (var i = 0; i < positions.Length; i++)
            {
                Gizmos.matrix = Matrix4x4.TRS(positions[i], Quaternion.identity, Vector3.one);
                if (i != 0)
                    Gizmos.DrawLine(Vector3.zero , positions[i - 1] - positions[i]);

                Gizmos.matrix *= Matrix4x4.TRS(Vector3.zero, rotations[i], Vector3.one);
                if (wire)
                {
                    Gizmos.DrawWireSphere(Vector3.zero, radius);
                }
                else
                {
                    Gizmos.DrawSphere(Vector3.zero, radius);
                }
                
            }
        }
    }

    private void TransformChainWithPose(ChainPose chain, out Vector3[] result, out Quaternion[] rotations)
    {
        result = new Vector3[_joins.Length];
        rotations = new Quaternion[_joins.Length];

        var transformMatrix = Matrix4x4.identity;
        var chainRootParentTransform = _joins[0].parent;

        if(chainRootParentTransform != null)
            transformMatrix = Matrix4x4.TRS(chainRootParentTransform.position, chainRootParentTransform.rotation, chainRootParentTransform.lossyScale);

        if (_initPositions.Length == 0 || _initRotations.Length == 0)
            Initialize();

        for (var i = 0; i < _joins.Length; i++)
        {
            rotations[i] = _initRotations[i] * chain.GetRotation(i);

            transformMatrix = transformMatrix * Matrix4x4.TRS(
                _initPositions[i],
                rotations[i],
                Vector3.one);

            result[i] = transformMatrix.MultiplyPoint(Vector3.zero);
        }
    }

    private ChainPose InterpolatePoses()
    {
        var cp = ScriptableObject.CreateInstance<ChainPose>();
        if (_leftPose.JoinCount != _rightPose.JoinCount || _upPose.JoinCount != _downPose.JoinCount || _leftPose.JoinCount != _downPose.JoinCount)
        {
            Debug.LogError("Poses count don't match");
            return cp;
        }


        var lerpedPoses = new Vector3[_leftPose.JoinCount];
        for (var i = 0; i < _leftPose.JoinCount; i++)
        {
            var horizontalLerped = Vector3.Lerp(
                _leftPose.GetEulerAngles(i),
                _rightPose.GetEulerAngles(i),
                _horizontalLerp * 0.5f + 0.5f);

            var verticalLerped = Vector3.Lerp(
                _upPose.GetEulerAngles(i),
                _downPose.GetEulerAngles(i),
                _verticalLerp * 0.5f + 0.5f);

            lerpedPoses[i] = horizontalLerped + verticalLerped;
        }
        
        cp.Initialize(lerpedPoses);
        return cp;
    }
}
