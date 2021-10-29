using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HummingbirdController : MonoBehaviour
{
    [Header("Controller input")]
    [SerializeField] [Range(-1, 1)] private float _strength;
    private const float FORCE_MULTIPLIER = 10000;

    [Space(5)]
    [SerializeField] [Range(-1f, 1f)] private float _horizontalControl;
    [SerializeField] [Range(-1f, 1f)] private float _verticalControl;
    [Space(5)]
    [SerializeField] [Range(-1f, 1f)] private float _lastJoinWeigth;
    private const float LAST_WEIGTH_MIN = 0.125f;
    private const float LAST_WEIGTH_MAX = 1.25f;


    [Space(20)]
    [SerializeField] private Transform _leftWingTransformReference;
    [SerializeField] private Transform _rightWingTransformReference;

    [Space(20)]
    [SerializeField] private CentroidComputer _centroidComputer;
    [SerializeField] private ChainTargetController _chainController;
    [SerializeField] private Animator _animator;

    private Vector3 _leftWingCentroidDelta;
    private float _leftWingCentroidDistance;
    private Vector3 _leftForce;

    private Vector3 _rightWingCentroidDelta;
    private float _rightWingCentroidDistance;
    private Vector3 _rightForce;

    private Rigidbody _rigidbody;

    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();
        
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 30;
    }

    private void LateUpdate()
    {
        _chainController.SetChainControlsValues(_horizontalControl, _verticalControl);

        var normalizedWeight = _lastJoinWeigth * 0.5f + 0.5f;
        _centroidComputer.OverrideWeight(7, Mathf.Lerp(LAST_WEIGTH_MIN, LAST_WEIGTH_MAX, normalizedWeight));
        _animator.SetFloat("OpenTail", normalizedWeight);

        _leftWingCentroidDelta = _leftWingTransformReference.position - _centroidComputer.Centroid;
        _leftWingCentroidDistance = _leftWingCentroidDelta.magnitude;
        
        _rightWingCentroidDelta = _rightWingTransformReference.position - _centroidComputer.Centroid;
        _rightWingCentroidDistance = _rightWingCentroidDelta.magnitude;

        _leftForce = ForceDirection(_leftWingCentroidDelta, _leftWingCentroidDistance);
        _rightForce = ForceDirection(_rightWingCentroidDelta, _rightWingCentroidDistance);

        if (_rigidbody != null)
        {
            _rigidbody.angularDrag = Mathf.Lerp(35f, 45f, normalizedWeight);
            _rigidbody.drag = Mathf.Lerp(2f, 3.5f, normalizedWeight);
            var addedForceDrag = Mathf.Lerp(500, -500, normalizedWeight);

            var normalizedStrength = _strength * 0.5f + 0.5f;
            _rigidbody.AddForceAtPosition(
                (FORCE_MULTIPLIER + addedForceDrag) * _leftForce * normalizedStrength * Time.deltaTime,
                _leftWingTransformReference.position);

            _rigidbody.AddForceAtPosition(
                (FORCE_MULTIPLIER + addedForceDrag) * _rightForce * normalizedStrength * Time.deltaTime, 
                _rightWingTransformReference.position);
        }
    }

    private Vector3 ForceDirection(Vector3 centroidDelta, float centroidDistance)
    {
        var forceDirection = Vector3.ProjectOnPlane(transform.up, centroidDelta).normalized;
        forceDirection = Vector3.Reflect(forceDirection, transform.forward);
        forceDirection *= centroidDistance;
        return forceDirection;
    }


    /*
    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 500, 500));
        GUILayout.BeginVertical();
        GUILayout.Label($"Left {_leftForce.x * 1000}, {_leftForce.y * 1000}, {_leftForce.z * 1000}");
        GUILayout.Label($"Right {_rightForce.x * 1000}, {_rightForce.y * 1000}, {_rightForce.z * 1000}");
        GUILayout.EndVertical();
        GUILayout.EndArea();

    }
    */
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            LateUpdate();

        var normalizedStrength = _strength * 0.5f + 0.5f;

        Gizmos.color = new Color(0, 125, 125);
        Gizmos.matrix = Matrix4x4.TRS(_centroidComputer.Centroid, Quaternion.identity, Vector3.one);

        Gizmos.DrawLine(
            _leftWingCentroidDelta,
            _leftWingCentroidDelta + FORCE_MULTIPLIER * _leftForce * normalizedStrength * 0.00025f);

        Gizmos.matrix *= Matrix4x4.TRS(Vector3.zero, Quaternion.LookRotation(_leftWingCentroidDelta, transform.forward), Vector3.one);
        //Gizmos.DrawWireSphere(Vector3.zero, _leftWingCentroidDistance);

        Gizmos.matrix = Matrix4x4.TRS(_centroidComputer.Centroid, Quaternion.identity, Vector3.one);

        Gizmos.DrawLine(
               _rightWingCentroidDelta,
               _rightWingCentroidDelta + FORCE_MULTIPLIER * _rightForce * normalizedStrength * 0.00025f);

        Gizmos.matrix *= Matrix4x4.TRS(Vector3.zero, Quaternion.LookRotation(_rightWingCentroidDelta, transform.forward), Vector3.one);
        //Gizmos.DrawWireSphere(Vector3.zero, _rightWingCentroidDistance);
    }
}
