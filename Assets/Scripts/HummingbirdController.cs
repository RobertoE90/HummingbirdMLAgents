using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class HummingbirdController : MonoBehaviour
{
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

    private Rigidbody _rigidbody;
    private Quaternion _wingsInitialRotation;

    private bool _freezed = false;

    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();
        _wingsInitialRotation = _wingRoot.localRotation;
        _environmentController.ConfigureEnvironment();
    }

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

        UpdateHumingbirdFromInput();
        _chainController.ForceUpdatePose();
        UpdateRigidBodyCenter();

        yield return new WaitForFixedUpdate();
        _freezed = false;
    }

    private void Update()
    {
       UpdateHumingbirdFromInput();   
    }

    private void UpdateHumingbirdFromInput()
    {
        _chainController.SetChainControlsValues(_horizontalTailControl, _verticalTailControl);

        var normalizedOpenTailValue = _openTail * 0.5f + 0.5f;
        _animator.SetFloat("OpenTail", normalizedOpenTailValue);

        _centroidComputer.OverrideWeight(8, Mathf.Lerp(OPEN_TAIL_WEIGTH_A, OPEN_TAIL_WEIGTH_B, normalizedOpenTailValue));

        _wingRoot.localRotation = _wingsInitialRotation * Quaternion.Euler(
            Mathf.Lerp(-20f, 20f, _verticalWingControl * 0.5f + 0.5f),
            0,
            Mathf.Lerp(-10f, 10f, _horizontalWingControl * 0.5f + 0.5f));

        _rigidbody.angularDrag = Mathf.Lerp(35, 45f, normalizedOpenTailValue);
        _rigidbody.drag = Mathf.Lerp(2f, 3.5f, normalizedOpenTailValue);
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

            var forceVector = _wingForceReferences[i].up;
            forceVector = FORCE_MULTIPLIER * forceVector * normalizedStrength;
            _rigidbody.AddForceAtPosition(
                forceVector,
                forceWorldPoint);

            Debug.DrawRay(forceWorldPoint, forceVector * 0.025f, Color.blue);

            var centerDelta = forceWorldPoint - _rigidbody.worldCenterOfMass;
            Debug.DrawRay(forceWorldPoint, Vector3.ProjectOnPlane(forceVector, centerDelta.normalized) * 0.03f, Color.green);

            Debug.DrawLine(
                _rigidbody.worldCenterOfMass,
                forceWorldPoint,
                Color.red);
        }
    }

    private void OnTriggerExit(Collider other) //the only trigger in scene is the environment bounds
    {
        if (_freezed)
            return;
        Debug.Log("Restart episode");
        _environmentController.ConfigureEnvironment();
    }

    private void OnDrawGizmos()
    {
        if (!_rigidbody)
            return;
            
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(_rigidbody.worldCenterOfMass, 0.025f);
    }
}
