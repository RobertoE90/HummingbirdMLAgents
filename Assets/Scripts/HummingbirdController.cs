using System.Collections;
using Unity.Barracuda;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class HummingbirdController : Agent
{
    [SerializeField] private bool _isTraining;
    [SerializeField] private bool _debug = true;
    [SerializeField] private bool _debugText = true;

    [Header("Controller input")]
    [SerializeField] [Range(-1, 1)] private float _strength;
    private const float FORCE_MULTIPLIER = 4.905f;

    [Space(20)]
    [SerializeField] [Range(-1f, 1f)] private float _horizontalHeadControl;
    [SerializeField] [Range(-1f, 1f)] private float _verticalHeadControl;
    [SerializeField] [Range(-1f, 1f)] private float _holdHeadPoseSlider;

    [Space(20)]
    [SerializeField] [Range(-1f, 1f)] private float _horizontalWingControl;
    [SerializeField] [Range(-1f, 1f)] private float _verticalWingControl;

    [SerializeField] [Range(-1f, 1f)] private float _horizontalTailControl;
    [SerializeField] [Range(-1f, 1f)] private float _verticalTailControl;
    [Space(20)]
    [SerializeField] [Range(-1f, 1f)] private float _openTail;
    private const float OPEN_TAIL_WEIGTH_A = 0.15f;
    private const float OPEN_TAIL_WEIGTH_B = 0.75f;

    [Space(20)]
    [SerializeField] private Transform[] _wingForceReferences;
    [SerializeField] private Transform _wingRoot;
    [SerializeField] private Transform _headTransform;
    [SerializeField] private Transform _root;
    [SerializeField] private Transform _tailTip;
    [SerializeField] private ExitTriggererRaizer _exitTriggererRaizer;

    [Space(20)]
    [SerializeField] private CentroidComputer _centroidComputer;
    [SerializeField] private ChainTargetController _chainController;
    [SerializeField] private Animator _animator;
    [SerializeField] private HeadNeckController _headNeckController;

    [Header("External references")]
    [SerializeField] private EnvironmentController _environmentController;

    [Header("Brains")]
    [SerializeField] private NNModel _goToTargetBrain;
    [SerializeField] private NNModel _hoverBrain;

    private float _currentHorizontalTailControl;
    private float _currentVerticalTailControl;
    private float _currentOpenTailControl;

    private Rigidbody _rigidbody;
    private Vector3 _previusVelocity = Vector3.zero;
    private Quaternion _wingsInitialRotation;
    private Vector3 _initialRootRelativePosition;
    private Quaternion _initialRootRelativeRotation;
    private float _rootToHeadDefaultDistance;

    private Vector3 _lastFrameHeadPosition;
    private Vector3 _headVelocity;

    private bool _freezed = false;
    private float _inRangeEnterTime = -1f;
    private bool _isOnTarget = false;
    private bool _isOnDistance = false;
    private float _inRangeTargetTime = 10f;
    private const float _maxInRangeSpeed = 1.25f;

    public bool IsOnRange => _isOnTarget;

    private float _elapsedTime;
    private float _lastEpisodeReward;
    private float _currentStepReward;


    public void Repose(Vector3 newPosition)
    {
        StartCoroutine(ReposeCoroutine(newPosition));
    }

    private IEnumerator ReposeCoroutine(Vector3 newPosition)
    {
        if (!_rigidbody)
            _rigidbody = _root.GetComponent<Rigidbody>();

        _freezed = true;

        _headNeckController.ImmediateSetHeatPose(_horizontalHeadControl, _verticalHeadControl, _holdHeadPoseSlider);

        yield return new WaitForFixedUpdate();

        transform.localPosition = newPosition;
        transform.rotation = Quaternion.identity;

        _root.localPosition = _initialRootRelativePosition;
        _root.localRotation = _initialRootRelativeRotation;

        //Default state
        _strength = 0f;

        _horizontalHeadControl = 1f;
        _verticalHeadControl = 0.3f;
        _holdHeadPoseSlider = -1f;

        _verticalWingControl = 0f;
        _horizontalWingControl = 0f;

        _horizontalTailControl = -0.0109999f;
        _verticalTailControl = 0.408085f;

        _openTail = -1;

        UpdateHumingbirdFromInput(false);
        _chainController.ForceUpdatePose();
        UpdateRigidBodyCenter();


        yield return new WaitForFixedUpdate();

        _rigidbody.velocity = Vector3.zero;
        _rigidbody.angularVelocity = Vector3.zero;

        _freezed = false;
    }

    #region ML_AGENTS_IMPLEMENTATION
    public override void Initialize()
    {
        _rigidbody = _root.GetComponent<Rigidbody>();
        _wingsInitialRotation = _wingRoot.localRotation;

        _initialRootRelativePosition = _root.localPosition;
        _initialRootRelativeRotation = _root.localRotation;
        _rootToHeadDefaultDistance = (_headTransform.position - _root.position).magnitude;

        _headNeckController.Initialize();
        _exitTriggererRaizer.ExitedTrigger += OnExitedEnvironmentVolume;

        this.MaxStep = _isTraining ? 2000 : 0;
        _environmentController.ConfigureEnvironment(true);
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(_root.rotation); //4
        sensor.AddObservation(_headVelocity); //3
        sensor.AddObservation(_rigidbody.angularVelocity); //3
        
        var toTargetVector = _environmentController.TargetPosition - _root.position;

        var toTargetVectorMagnitude = toTargetVector.magnitude;
        if (toTargetVectorMagnitude != 0)
        {
            sensor.AddObservation(toTargetVector / toTargetVectorMagnitude); //3
            sensor.AddObservation(Vector3.Dot(toTargetVector / toTargetVectorMagnitude, _headTransform.forward * -1)); //1
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(1);
        }

        sensor.AddObservation(_rigidbody.centerOfMass); //3
        sensor.AddObservation(Vector3.Dot(_root.up, Vector3.up)); //1
        sensor.AddObservation(Vector3.Dot(_environmentController.TargetForward, _root.forward * -1)); //1
        sensor.AddObservation(Vector3.Dot(_environmentController.TargetForward, _headTransform.forward * -1)); //1
        sensor.AddObservation(Mathf.Clamp01(toTargetVectorMagnitude / 2f)); //1
    }

    private void FrameRewards()
    {
        _inRangeTargetTime = Academy.Instance.EnvironmentParameters.GetWithDefault("in_range_time", 10);
        _inRangeTargetTime = Random.Range(2, 7);
        var headToTarget = (_environmentController.TargetPosition - _headTransform.position);

        var distance = headToTarget.magnitude;
        var velocityMagnitude = _headVelocity.magnitude;

        var currentFrameInDistance = (distance < _environmentController.TargetPositionRadius);
        var lookingToTarget = Vector3.Dot(_environmentController.TargetForward, _root.forward * -1) > 0.8f;
        
        var inTarget = currentFrameInDistance && (velocityMagnitude < _maxInRangeSpeed) && lookingToTarget;



        if (!_isTraining)
        {
            if (currentFrameInDistance)
            {
                if (!_isOnTarget)//entered on target
                {
                    _inRangeEnterTime = Time.time;
                }

                _isOnTarget = true;

                _isOnDistance = true; //we only consider that the bird is on distance if it enter the valid range

                _elapsedTime = Time.time - _inRangeEnterTime;

                //if (_elapsedTime >= _inRangeTargetTime)
                  //  _environmentController.ReposeTarget();
            }
            else
            {
                if (_isOnTarget) //exited on target
                {
                    _isOnTarget = false;
                }
            }

            if (currentFrameInDistance)
                SetModel("HummingBird", _hoverBrain, Unity.MLAgents.Policies.InferenceDevice.GPU);
            if (distance > _environmentController.TargetPositionRadius * 2f)
                SetModel("HummingBird", _goToTargetBrain, Unity.MLAgents.Policies.InferenceDevice.GPU);

            return;
        }

        //if (currentFrameInDistance && (velocityMagnitude > _maxInRangeSpeed))
        if (currentFrameInDistance && lookingToTarget && (velocityMagnitude > _maxInRangeSpeed))
        {
            DoEndEpisode(1f, false, false);
        }


        if (_isTraining)
        {
            _currentStepReward = 0;
            _currentStepReward += ((inTarget) ? 1 : -1) / (float)MaxStep;

            //reward for having the head aligned with the target forward
            _currentStepReward +=1.25f * Vector3.Dot(_environmentController.TargetForward, _headTransform.forward * -1) / (float)MaxStep;
            _currentStepReward += 0.75f * Vector3.Dot(_environmentController.TargetForward, _root.forward * -1) / (float)MaxStep;
            _currentStepReward += 0.5f * Vector3.Dot(_root.up, Vector3.up) / (float)MaxStep;
            

            AddReward(_currentStepReward);
        }
    }

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = _strength;

        actionsOut[1] = _horizontalHeadControl;
        actionsOut[2] = _verticalHeadControl;
        actionsOut[3] = _holdHeadPoseSlider;

        actionsOut[4] = _horizontalWingControl;
        actionsOut[5] = _verticalWingControl;

        actionsOut[6] = _horizontalTailControl;
        actionsOut[7] = _verticalTailControl;
        
        actionsOut[8] = _openTail;
    }

    public override void OnActionReceived(float[] vectorAction) {
        _strength = vectorAction[0];

        _horizontalHeadControl = vectorAction[1];
        _verticalHeadControl = vectorAction[2];
        _holdHeadPoseSlider = vectorAction[3];

        _horizontalWingControl = vectorAction[4];
        _verticalWingControl = vectorAction[5];

        _horizontalTailControl = vectorAction[6];
        _verticalTailControl = vectorAction[7];
        _openTail = vectorAction[8];

        //Update agent
        UpdateHumingbirdFromInput(true);
        ComputeHeadVelocity();
        FrameRewards();

        if (_isTraining && StepCount >= MaxStep - 1)
            DoEndEpisode(-1, true, true);
    }
    

    private void OnExitedEnvironmentVolume() //the only trigger in scene is the environment bounds
    {
        if (_freezed || !_isTraining)
            return;

        DoEndEpisode(-1f, true, true);
    }

    private void DoEndEpisode(float reward, bool failed, bool repositionateBird)
    {
        AddReward(reward);
        _environmentController.EndEpisodeFeedback(failed);
        _environmentController.ConfigureEnvironment(repositionateBird);
        _isOnTarget = false;
        _isOnDistance = false;
        _lastEpisodeReward = GetCumulativeReward();
        EndEpisode();
    }
    #endregion


    private void UpdateHumingbirdFromInput(bool lerp)
    {
        if (lerp)
        {
            _currentHorizontalTailControl = Mathf.Lerp(_currentHorizontalTailControl, _horizontalTailControl, 7 * Time.deltaTime);
            _currentVerticalTailControl = Mathf.Lerp(_currentVerticalTailControl, _verticalTailControl, 7 * Time.deltaTime);
            _currentOpenTailControl = Mathf.Lerp(_currentOpenTailControl, _openTail, 8 * Time.deltaTime);
        }
        else
        {
            _currentHorizontalTailControl = _horizontalTailControl;
            _currentVerticalTailControl = _verticalTailControl;
            _currentOpenTailControl = _openTail;
        }

        _headNeckController.SetTargetHeadPoseParams(_horizontalHeadControl, _verticalHeadControl, _holdHeadPoseSlider);
        _chainController.SetChainControlsValues(_currentHorizontalTailControl, _currentVerticalTailControl);


        var normalizedOpenTailValue = _currentOpenTailControl * 0.5f + 0.5f;
        _animator.SetFloat("OpenTail", normalizedOpenTailValue);

        _centroidComputer.OverrideWeight(8, Mathf.Lerp(OPEN_TAIL_WEIGTH_A, OPEN_TAIL_WEIGTH_B, normalizedOpenTailValue));

        _wingRoot.localRotation = _wingsInitialRotation * Quaternion.Euler(
            Mathf.Lerp(-30f, 30f, _verticalWingControl * 0.5f + 0.5f),
            0,
            Mathf.Lerp(-15f, 15f, _horizontalWingControl * 0.5f + 0.5f));

        if (_holdHeadPoseSlider > 0)
            ReAlignBody();
    }

    private void ComputeHeadVelocity()
    {
        _headVelocity = _headTransform.position - _lastFrameHeadPosition;
        _headVelocity /= Time.deltaTime;
        _lastFrameHeadPosition = _headTransform.position;
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
                Debug.DrawLine(
                    _rigidbody.worldCenterOfMass,
                    forceWorldPoint,
                    Color.red);
            }
        }

        //tail force
        var normalizedOpenTailValue = _currentOpenTailControl * 0.5f + 0.5f;
        var acceleration = _rigidbody.velocity - _previusVelocity;
        acceleration /= Time.fixedDeltaTime;
        var windForce = acceleration * _rigidbody.mass;
        windForce *= -1;
        
        var headToTailVector = _tailTip.position - _headTransform.position;
        headToTailVector = headToTailVector.normalized;
        
        var taleNormal = Vector3.Cross(headToTailVector, _tailTip.right);
        var dot = Vector3.Dot(taleNormal, windForce);

        var appliedForce = dot * taleNormal * Mathf.Lerp(0.15f, 0.5f, normalizedOpenTailValue);
        _previusVelocity = _rigidbody.velocity;
        var tailForcePoint = (_tailTip.position + _tailTip.parent.position) * 0.5f;

        _rigidbody.AddForceAtPosition(appliedForce, tailForcePoint);
        if (_debug)
            Debug.DrawRay(tailForcePoint, appliedForce * 0.05f, Color.magenta);
    }

    private void UpdateRigidBodyCenter()
    {
        _centroidComputer.ComputeCentroidPosition();
        var pivot = _centroidComputer.Centroid - _root.position;
        pivot = _root.InverseTransformDirection(pivot + _rigidbody.velocity * Time.fixedDeltaTime);
        _rigidbody.centerOfMass = pivot;
    }

    private void ReAlignBody()
    {
        var targetPosition = _headTransform.position + _headTransform.forward * _rootToHeadDefaultDistance;
        _rigidbody.MovePosition(Vector3.Lerp(_root.position, targetPosition, 10 * Time.deltaTime));
        var targetRotation = Quaternion.LookRotation(_root.position - _headTransform.position, _root.up);
        _rigidbody.MoveRotation(Quaternion.Lerp(_root.rotation, targetRotation, 10 * Time.deltaTime));
    }

    private void OnGUI()
    {
        if (!_debugText)
            return;

        GUI.skin.label.fontSize = 20;
        GUILayout.BeginArea(new Rect(20, 20, 400, 500));
        GUILayout.BeginVertical();
        GUILayout.Label($"Velocity {_headVelocity.magnitude}");
        GUILayout.Label($"Target in range time {_inRangeTargetTime}");
        GUILayout.Label($"Reward {GetCumulativeReward()}");
        GUILayout.Label($"Last episode reward {_lastEpisodeReward}");
        GUILayout.Label($"In Range time {_elapsedTime}");

        GUILayout.EndVertical();
        GUILayout.EndArea();
    }

    private void OnDrawGizmos()
    {
        if (!_rigidbody || !_debug)
            return;
            
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(_rigidbody.worldCenterOfMass, 0.025f);
    }
}
