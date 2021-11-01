using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChainToTarget : MonoBehaviour
{
    [SerializeField] private Transform[] _joins;
    [SerializeField] private Transform _alignUpVector;

    
    public void FitChainToPositions(Vector3[] targetPositions)
    {
        if (_joins.Length != targetPositions.Length)
            return;

        for (var j = 0; j < 5; j++)
        {
            _joins[0].position = targetPositions[0];
            for (var i = 0; i < _joins.Length - 1; i++)
            {
                var delta = _joins[i].position - targetPositions[i + 1];
                _joins[i].rotation = Quaternion.LookRotation(
                    delta,
                    _alignUpVector.position - transform.position);
            }
        }
    }
}
