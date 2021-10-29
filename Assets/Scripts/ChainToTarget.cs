using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChainToTarget : MonoBehaviour
{
    [SerializeField] private Transform[] _joins;
    [SerializeField] private Transform _alignUpVector;

    private Vector3[] _targetPositions;
    public Vector3[] TargetPositions
    {
        set { _targetPositions = value; }
    }

    private void Update()
    {
        if (_targetPositions == null || _joins.Length != _targetPositions.Length)
            return;

        _joins[0].position = _targetPositions[0];
        
        for (var i = 0; i < _joins.Length - 1; i++)
        {
            var delta = _joins[i].position - _targetPositions[i + 1];
            _joins[i].rotation = Quaternion.LookRotation(
                delta,
                _alignUpVector.position - transform.position) ;
        }
    }
}
