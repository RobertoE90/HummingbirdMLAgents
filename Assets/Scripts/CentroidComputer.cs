using UnityEngine;

public class CentroidComputer : MonoBehaviour
{
    [SerializeField] private float[] _weights;
    [SerializeField] private Transform[] _joins;
    [Space(20)]
    [SerializeField] private bool _debug;

    private Vector3 _centroid;
    public Vector3 Centroid => _centroid;

    private Vector3[] _positions;

    public void Update()
    {
        if (_weights.Length != _joins.Length)
        {
            Debug.LogError("Missmatch on sizes");
            return;
        }

        if (_positions == null)
            _positions = new Vector3[_joins.Length];

        for (var i = 0; i < _joins.Length; i++)
        {
            _positions[i] = _joins[i].position;
        }

        ComputeCentroidPosition();
    }

    public void OverrideWeight(int index, float value)
    {
        _weights[index] = value;
    }

    private void ComputeCentroidPosition()
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
            var p = _positions[i];

            div += w;
            centroid += p * w;
        }

        if (div != 0)
            _centroid = centroid / div;
        else
            _centroid = _positions[0];
    }

    private void OnDrawGizmos()
    {
        if (!_debug)
            return;

        if (!Application.isPlaying)
            Update();

        if (_weights.Length != _joins.Length)
        {
            Debug.LogError("Missmatch on sizes");
            return;
        }

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(_centroid, 0.015f);

        Gizmos.color = Color.magenta;
        for (var i = 0; i < _weights.Length; i++)
        {
            Gizmos.DrawWireSphere(_positions[i], _weights[i] * 0.025f);
        }
    }
}
