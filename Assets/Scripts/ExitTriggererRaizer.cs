using System;
using UnityEngine;

public class ExitTriggererRaizer : MonoBehaviour
{
    public event Action ExitedTrigger;

    private void OnTriggerExit(Collider other) //the only trigger in scene is the environment bounds
    {
        ExitedTrigger?.Invoke();
    }
}
