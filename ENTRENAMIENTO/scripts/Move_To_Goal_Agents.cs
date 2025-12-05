using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
public class Move_To_Goal_Agents : Agent
{
    [SerializeField] private Transform TargetTransformPosition;

    [SerializeField] private Transform Target;
    [SerializeField] private float movespeed;

    [SerializeField] private Material win;
    [SerializeField] private Material lose;

    [SerializeField] private MeshRenderer myrenderer;

    public override void OnEpisodeBegin()
    {
        this.transform.localPosition = new Vector3(0,1.5f,0);
    }

    
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(this.transform.localPosition);
        sensor.AddObservation(Target.localPosition);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        
        transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * movespeed;

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<Goal>(out Goal goal))
        {
            SetReward(+1f);
            myrenderer.material = win;
            EndEpisode();
        }
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            SetReward(-1f);
            myrenderer.material = lose;
            EndEpisode();
        }

    }
}
