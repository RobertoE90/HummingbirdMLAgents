using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(BoxCollider))]
public class EnvironmentController : MonoBehaviour
{
    [SerializeField] private Vector3 _environmentSize;
    [Header("Scene References")]
    [SerializeField] private HummingbirdController _hummingbird;
    [SerializeField] private Transform _targetPosition;
    [SerializeField] private float _targetPositionRadius;
    [SerializeField] private GameObject _floor;
    [SerializeField] private BoxCollider _boxCollider;

    public Vector3 TargetPosition => _targetPosition.position;
    public float TargetPositionRadius => _targetPositionRadius;


    public void ConfigureEnvironment()
    {
        _boxCollider.size = _environmentSize;
        _boxCollider.center = Vector3.up * _environmentSize.y * 0.5f;
        _floor.transform.localPosition = Vector3.up * -0.1f;
        _floor.transform.localScale = new Vector3(_environmentSize.x, 0.2f, _environmentSize.z);

        _hummingbird.Repose(Vector3.up * _environmentSize.y * 0.5f);

        var random = new Vector3(Random.value, Random.value, Random.value);
        random.x = Mathf.Clamp(random.x, 0.2f, random.x);
        random.y = Mathf.Clamp(random.y, 0.2f, random.y);
        random.z = Mathf.Clamp(random.z, 0.2f, random.z);

        random = new Vector3(
            (random.x * 2 - 1f) * _environmentSize.x,
            (random.y * 2 - 1f) * _environmentSize.y,
            (random.z * 2 - 1f) * _environmentSize.z) * 0.5f;

        _targetPosition.localPosition = Vector3.up * _environmentSize.y * 0.5f + random * 0.45f;
    }


    private void OnDrawGizmos()
    {
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(_boxCollider.center, _boxCollider.size);
        Gizmos.DrawWireSphere(_targetPosition.position, _targetPositionRadius);
    }
}
