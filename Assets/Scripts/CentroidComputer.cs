using UnityEngine;

public class CentroidComputer : MonoBehaviour
{
    [SerializeField] private float[] _weights;
    [SerializeField] private Transform[] _joins;

    [SerializeField] private bool _debug;

    private Vector3 _centroid;
    public Vector3 Centroid => _centroid;


    public void OverrideWeight(int index, float value)
    {
        _weights[index] = value;
    }

    public void ComputeCentroidPosition()
    {
        if(_weights.Length != _joins.Length)
        {
            Debug.LogError("Missmatch on sizes");
            return;
        }

        var centroid = Vector3.zero;
        var div = 0f;

        for(var i = 0; i <_joins.Length; i++)
        {
            var w = _weights[i];
            var p = _joins[i].position;
            div += w;
            centroid += p * w;
        }

        if (div != 0)
            _centroid = centroid / div;
        else
            _centroid = _joins[0].position;
    }

    private void OnDrawGizmos()
    {
        if (!_debug)
            return;

        Gizmos.color = Color.red;
        for (var i = 0; i < _joins.Length; i++)
        {
            Gizmos.DrawWireSphere(_joins[i].position, _weights[i] * 0.05f);
        }
    }
}
