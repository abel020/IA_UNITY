using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class Move_Laberynt_Agent : Agent
{
    [Header("Movimiento")]
    [SerializeField] private float moveSpeed = 3f;

    [Header("Reward shaping A*")]
    [SerializeField] private bool useDistanceShaping = true;
    [SerializeField] private float rewardTowardsGoal = 0.02f;
    [SerializeField] private float penaltyAwayFromGoal = -0.02f;
    [SerializeField] private float timePenaltyPerStep = -0.001f;

    [Header("Feedback visual")]
    [SerializeField] private Material win;
    [SerializeField] private Material lose;
    [SerializeField] private MeshRenderer myrenderer;

    // Posición inicial de ESTE agente (la que le dio el MazeGenerator al instanciarlo)
    private Vector3 initialPosition;
    private int previousDistance = int.MaxValue;

    private void Start()
    {
        // Guardamos el spawn original que nos asignó MazeGenerator.SpawnPlayer(...)
        initialPosition = transform.position;

        if (MazeGenerator.Instance != null && MazeGenerator.Instance.HasAnyDistanceField)
        {
            previousDistance = MazeGenerator.Instance.GetDistanceFromWorldPos(transform.position);
        }
    }

    public override void OnEpisodeBegin()
    {
        // Volver al spawn original de este agente
        transform.position = initialPosition;

        // Reset de distancia previa usando el campo de distancias
        if (MazeGenerator.Instance != null && MazeGenerator.Instance.HasAnyDistanceField)
        {
            previousDistance = MazeGenerator.Instance.GetDistanceFromWorldPos(transform.position);
        }
        else
        {
            previousDistance = int.MaxValue;
        }

        // Reset visual opcional
        // if (myrenderer != null && neutralMaterial != null)
        //     myrenderer.material = neutralMaterial;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 agentPos = transform.position;

        Vector3 goalPos = agentPos;
        if (MazeGenerator.Instance != null)
        {
            goalPos = MazeGenerator.Instance.GetGoalWorldPos();
        }

        // Observaciones:
        // 1) posición del agente
        // 2) posición del objetivo
        // 3) vector relativo objetivo - agente (dirección hacia el goal)
        sensor.AddObservation(agentPos);
        sensor.AddObservation(goalPos);
        sensor.AddObservation(goalPos - agentPos);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        Vector3 movement = new Vector3(moveX, 0f, moveZ) * moveSpeed * Time.deltaTime;
        transform.position += movement;

        // Penalización pequeña por tiempo: evita que dé vueltas infinitas
        AddReward(timePenaltyPerStep);

        // Reward shaping usando el campo de distancias de A*
        if (useDistanceShaping && MazeGenerator.Instance != null && MazeGenerator.Instance.HasAnyDistanceField)
        {
            int currentDistance = MazeGenerator.Instance.GetDistanceFromWorldPos(transform.position);

            if (currentDistance != int.MaxValue && previousDistance != int.MaxValue)
            {
                if (currentDistance < previousDistance)
                {
                    // Se acercó al goal
                    AddReward(rewardTowardsGoal);
                }
                else if (currentDistance > previousDistance)
                {
                    // Se alejó del goal
                    AddReward(penaltyAwayFromGoal);
                }
            }

            previousDistance = currentDistance;
        }
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
            AddReward(+1f);
            if (myrenderer != null && win != null)
                myrenderer.material = win;

            EndEpisode();
        }

        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            AddReward(-1f);
            if (myrenderer != null && lose != null)
                myrenderer.material = lose;

            EndEpisode();
        }
    }
}
