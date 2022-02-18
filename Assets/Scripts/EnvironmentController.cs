using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using TMPro;
using UnityEngine;

[RequireComponent(typeof(BoxCollider))]
public class EnvironmentController : MonoBehaviour
{
    [SerializeField] private bool _debug;
    [SerializeField] private Vector3 _environmentSize;
    [Header("Scene References")]
    [SerializeField] private HummingbirdController _hummingbird;
    [SerializeField] private Transform _targetTransform;
    [SerializeField] private float _targetPositionRadius;
    [SerializeField] private Renderer _floor;
    [SerializeField] private BoxCollider _boxCollider;
    [SerializeField] private TextMeshPro _infoLabel;

    public Vector3 TargetPosition => _targetTransform.position;
    public float TargetPositionRadius => _targetPositionRadius;
    public Vector3 TargetForward => _targetTransform.forward;

    public async void EndEpisodeFeedback(bool failed)
    {
        _floor.sharedMaterial.SetColor("_BaseColor", failed ? Color.red : Color.blue);
        await Task.Delay(100);
        _floor.sharedMaterial.SetColor("_BaseColor", Color.green);
    }

    public void ConfigureEnvironment(bool reposeBird)
    {
        _boxCollider.size = _environmentSize;
        _boxCollider.center = Vector3.up * _environmentSize.y * 0.5f;
        _floor.transform.localPosition = Vector3.up * -0.1f;
        _floor.transform.localScale = new Vector3(_environmentSize.x, 0.2f, _environmentSize.z);

        if(reposeBird)
            _hummingbird.Repose(Vector3.up * _environmentSize.y * 0.5f);

        ReposeTarget();
     }

    public void ReposeTarget()
    {
        var random = new Vector3(Random.value, Random.value, Random.value);
        random.x = Mathf.Clamp(random.x, 0.2f, random.x);
        random.y = Mathf.Clamp(random.y, 0.2f, random.y);
        random.z = Mathf.Clamp(random.z, 0.2f, random.z);

        random = new Vector3(
            (random.x * 2 - 1f) * _environmentSize.x,
            (random.y * 2 - 1f) * _environmentSize.y,
            (random.z * 2 - 1f) * _environmentSize.z) * 0.5f;

        _targetTransform.localPosition = Vector3.up * _environmentSize.y * 0.5f + random * 0.45f;

        //_targetTransform.localPosition = Vector3.up * _environmentSize.y * 0.5f;
        //random rotate _targetTransform

        _targetTransform.localRotation =
            Quaternion.Euler(0, Random.Range(0, 360), 0) * Quaternion.Euler(Random.Range(-45, 45), 0, 0);
    }

    private void Update()
    {
        _infoLabel.text =
            $"Current reward {_hummingbird.GetCumulativeReward()}\n" +
            $"Steps {_hummingbird.StepCount} / {_hummingbird.MaxStep}";
        
    }

    private void OnDrawGizmos()
    {
        if (!_debug)
            return;

        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = Color.yellow;
        //Gizmos.DrawWireCube(_boxCollider.center, _boxCollider.size);

        Gizmos.color = _hummingbird.IsOnRange ? Color.blue : Color.yellow;
        Gizmos.matrix *= Matrix4x4.TRS(_targetTransform.localPosition, _targetTransform.localRotation, Vector3.one);
        Gizmos.DrawWireSphere(Vector3.zero, _targetPositionRadius);
        Gizmos.DrawWireCube(Vector3.forward * _targetPositionRadius * 0.5f, new Vector3(0.1f, 0.1f, 0.5f) * _targetPositionRadius);
    }
}
