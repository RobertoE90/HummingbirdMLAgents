using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class HummingbirdController : Agent
{
    [SerializeField] private bool _isTraining;
    [SerializeField] private bool _debug = true;

    [Header("Controller input")]
    [SerializeField] [Range(-1, 1)] private float _strength;
    private const float FORCE_MULTIPLIER = 4.905f;

    [Space(5)]
    [SerializeField] [Range(-1f, 1f)] private float _horizontalWingControl;
    [SerializeField] [Range(-1f, 1f)] private float _verticalWingControl;

    [SerializeField] [Range(-1f, 1f)] private float _horizontalTailControl;
    [SerializeField] [Range(-1f, 1f)] private float _verticalTailControl;
    [Space(5)]
    [SerializeField] [Range(-1f, 1f)] private float _openTail;
    private const float OPEN_TAIL_WEIGTH_A = 0.15f;
    private const float OPEN_TAIL_WEIGTH_B = 0.75f;

    [Space(20)]
    [SerializeField] private Transform[] _wingForceReferences;
    [SerializeField] private Transform _wingRoot;

    [Space(20)]
    [SerializeField] private CentroidComputer _centroidComputer;
    [SerializeField] private ChainTargetController _chainController;
    [SerializeField] private Animator _animator;
    [Header("External references")]
    [SerializeField] private EnvironmentController _environmentController;

    private float _currentHorizontalTailControl;
    private float _currentVerticalTailControl;
    private float _currentOpenTailControl;


    private Rigidbody _rigidbody;
    private Quaternion _wingsInitialRotation;

    private bool _freezed = false;
    private bool _episodeTimeouted = true;

#region ML_AGENTS_IMPLEMENTATION
    public override void Initialize()
    {
        _rigidbody = GetComponent<Rigidbody>();
        _wingsInitialRotation = _wingRoot.localRotation;
        this.MaxStep = _isTraining ? 1500 : 0;
        _environmentController.ConfigureEnvironment();
    }

    public override void OnEpisodeBegin()
    {
        if (_episodeTimeouted)
            _environmentController.EndEpisodeFeedback(true);

        _episodeTimeouted = true;

        _environmentController.ConfigureEnvironment();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.rotation);
        sensor.AddObservation(_rigidbody.velocity);
        sensor.AddObservation(_rigidbody.angularVelocity);
        sensor.AddObservation(_environmentController.TargetPosition - transform.position);
    }

    private void FrameRewards()
    {
        if(_isTraining)
            AddReward(-1/ (float)MaxStep);

        var distance = (_environmentController.TargetPosition - transform.position).magnitude;
        if(distance < _environmentController.TargetPositionRadius)
        {
            AddReward(1);
            _episodeTimeouted = false;
            _environmentController.EndEpisodeFeedback(false);
            EndEpisode();
        }
    }

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = _strength;
        actionsOut[1] = _horizontalWingControl;
        actionsOut[2] = _verticalWingControl;
        actionsOut[3] = _horizontalTailControl;
        actionsOut[4] = _verticalTailControl;
        actionsOut[5] = _openTail;
    }

    public override void OnActionReceived(float[] vectorAction) {
        _strength = vectorAction[0];
        _horizontalWingControl = vectorAction[1];
        _verticalWingControl = vectorAction[2];
        _horizontalTailControl = vectorAction[3];
        _verticalTailControl = vectorAction[4];
        _openTail = vectorAction[5];
    }
    

    private void OnTriggerExit(Collider other) //the only trigger in scene is the environment bounds
    {
        if (_freezed)
            return;

        _episodeTimeouted = false;
        AddReward(-1);
        _environmentController.EndEpisodeFeedback(true);
        EndEpisode();
    }
    #endregion

    public void Repose(Vector3 newPosition)
    {
        StartCoroutine(ReposeCoroutine(newPosition));
    }

    private IEnumerator ReposeCoroutine(Vector3 newPosition)
    {
        if (!_rigidbody)
            _rigidbody = GetComponent<Rigidbody>();

        _freezed = true;
        
        yield return new WaitForFixedUpdate();
        transform.localPosition = newPosition;
        transform.rotation = Quaternion.identity;

        _rigidbody.velocity = Vector3.zero;
        _rigidbody.angularVelocity = Vector3.zero;

        //Default state
        _strength = 0f;
        _verticalWingControl = 0f;
        _horizontalWingControl = 0f;
        _horizontalTailControl = -0.0109999f;
        _verticalTailControl = 0.408085f;
        _openTail = -1;

        UpdateHumingbirdFromInput(false);
        _chainController.ForceUpdatePose();
        UpdateRigidBodyCenter();

        yield return new WaitForFixedUpdate();
        _freezed = false;
    }

    private void Update()
    {
       UpdateHumingbirdFromInput(true);
       FrameRewards();
    }

    private void UpdateHumingbirdFromInput(bool lerp)
    {
        if (lerp)
        {
            _currentHorizontalTailControl = Mathf.Lerp(_currentHorizontalTailControl, _horizontalTailControl, 4 * Time.deltaTime);
            _currentVerticalTailControl = Mathf.Lerp(_currentVerticalTailControl, _verticalTailControl, 4 * Time.deltaTime);
            _currentOpenTailControl = Mathf.Lerp(_currentOpenTailControl, _openTail, 4 * Time.deltaTime);
        }
        else
        {
            _currentHorizontalTailControl = _horizontalTailControl;
            _currentVerticalTailControl = _verticalTailControl;
            _currentOpenTailControl = _openTail;
        }

        _chainController.SetChainControlsValues(_currentHorizontalTailControl, _currentVerticalTailControl);

        var normalizedOpenTailValue = _currentOpenTailControl * 0.5f + 0.5f;
        _animator.SetFloat("OpenTail", normalizedOpenTailValue);

        _centroidComputer.OverrideWeight(8, Mathf.Lerp(OPEN_TAIL_WEIGTH_A, OPEN_TAIL_WEIGTH_B, normalizedOpenTailValue));

        _wingRoot.localRotation = _wingsInitialRotation * Quaternion.Euler(
            Mathf.Lerp(-20f, 20f, _verticalWingControl * 0.5f + 0.5f),
            0,
            Mathf.Lerp(-10f, 10f, _horizontalWingControl * 0.5f + 0.5f));

        _rigidbody.angularDrag = Mathf.Lerp(37.5f, 55f, normalizedOpenTailValue);
        _rigidbody.drag = Mathf.Lerp(2.5f, 4f, normalizedOpenTailValue);
    }

    private void UpdateRigidBodyCenter()
    {
        _centroidComputer.ComputeCentroidPosition();
        var pivot = _centroidComputer.Centroid - transform.position;
        pivot = transform.InverseTransformDirection(pivot + _rigidbody.velocity * Time.fixedDeltaTime);
        _rigidbody.centerOfMass = pivot;
    }

    private void FixedUpdate()
    {
        if (_freezed)
            return;

        UpdateRigidBodyCenter();

        var normalizedStrength = _strength * 0.5f + 0.5f;

        for (var i = 0; i < _wingForceReferences.Length; i++)
        {
            var forceWorldPoint = _wingForceReferences[i].position + _rigidbody.velocity * Time.fixedDeltaTime;
            var openTailForceDelta = Mathf.Lerp(0, -3, _currentOpenTailControl * 0.5f - 0.5f);

            var forceVector = _wingForceReferences[i].up;
            forceVector = (FORCE_MULTIPLIER + openTailForceDelta) * forceVector * normalizedStrength;
            _rigidbody.AddForceAtPosition(
                forceVector,
                forceWorldPoint);

            if (_debug)
            {
                Debug.DrawRay(forceWorldPoint, forceVector * 0.025f, Color.blue);

                var centerDelta = forceWorldPoint - _rigidbody.worldCenterOfMass;
                Debug.DrawRay(forceWorldPoint, Vector3.ProjectOnPlane(forceVector, centerDelta.normalized) * 0.03f, Color.green);

                Debug.DrawLine(
                    _rigidbody.worldCenterOfMass,
                    forceWorldPoint,
                    Color.red);
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (!_rigidbody || !_debug)
            return;
            
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(_rigidbody.worldCenterOfMass, 0.025f);
    }
}
