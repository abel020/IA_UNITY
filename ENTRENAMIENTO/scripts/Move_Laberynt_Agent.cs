using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;

public class Move_Laberynt_Agent : Agent
{
    [Header("Movimiento")]
    [SerializeField] private float moveSpeed = 3f;
    [SerializeField] private float turnSpeed = 200f; // grados por segundo

    [Header("Reward shaping A* (distancia al objetivo)")]
    [SerializeField] private bool useDistanceShaping = true;
    [SerializeField] private float rewardTowardsGoal = 0.01f;
    [SerializeField] private float penaltyAwayFromGoal = -0.01f;
    [SerializeField] private float timePenaltyPerStep = -0.0005f;
    [SerializeField] private float goalReward = 2f;
    [SerializeField] private float wallPenalty = -1f;

    [Header("Reward A* Path (seguir el camino óptimo)")]
    [Tooltip("Si está activado, da una recompensa extra por mantenerse cerca del camino A* del laberinto.")]
    [SerializeField] private bool usePathShaping = true;
    [Tooltip("Distancia máxima (en unidades del mundo) para considerar que el agente está 'sobre' el camino A*.")]
    [SerializeField] private float pathNearThreshold = 1.5f;
    [Tooltip("Recompensa extra por paso cuando el agente está cerca del camino A*.")]
    [SerializeField] private float rewardOnPath = 0.02f;

    [Header("Anti-idle (evitar que se quede parado)")]
    [SerializeField] private bool useAntiIdle = true;
    [SerializeField] private float minMoveDistance = 0.02f;        // Distancia mínima que debe moverse para considerar que se movió
    [SerializeField] private int maxStepsWithoutMoving = 40;       // Nº de steps seguidos casi sin moverse
    [SerializeField] private float idlePenalty = -0.1f;            // Penalización si se queda mucho tiempo quieto

    [Header("Anti-loop (no progresa hacia la meta)")]
    [SerializeField] private bool useAntiLoop = true;
    [Tooltip("Pasos permitidos sin mejorar la distancia A* al objetivo.")]
    [SerializeField] private int maxStepsWithoutProgress = 50;
    [Tooltip("Penalización al detectar que no progresa hacia la meta.")]
    [SerializeField] private float noProgressPenalty = -0.2f;

    [Header("Observaciones extra")]
    [Tooltip("Si está activado, agrega una observación escalar con la distancia normalizada al objetivo (campo de distancias A*).")]
    [SerializeField] private bool includeDistanceObservation = true;
    [Tooltip("Factor para normalizar la distancia discreta. Distancia_normalizada = clamp01(dist / factor).")]
    [SerializeField] private float distanceNormalizationFactor = 50f;

    [Header("Feedback visual")]
    [SerializeField] private Material win;
    [SerializeField] private Material lose;
    [SerializeField] private Material neutral;
    [SerializeField] private MeshRenderer agentRenderer;

    // Floors de ESTE laberinto (ni inicio, ni fin, ni path óptimo)
    private List<MeshRenderer> mazeFloorRenderers = new List<MeshRenderer>();

    // Posición inicial de ESTE agente
    private Vector3 initialPosition;
    private bool hasInitialPosition = false;

    // Distancia discreta previa (campo de distancias A*)
    private int previousDistance = int.MaxValue;

    // Índice del laberinto al que pertenezco
    [SerializeField] private int mazeIndex = -1;

    // Parámetros de comportamiento (para saber si está en modo heurístico/teclado)
    private BehaviorParameters behaviorParameters;

    // Cache del camino A* de ESTE laberinto (en posiciones de mundo)
    private List<Vector3> cachedPathWorldPositions;
    private bool hasPathCache = false;

    // Anti-idle
    private Vector3 lastPosition;
    private int stepsSinceMove = 0;

    // Anti-loop (sin progreso)
    private int stepsSinceProgress = 0;

    // Flags de logging
    private bool loggedVectorSize = false;
    private bool loggedRayComponents = false;
    private bool loggedAgentSensors = false;

    public void SetMazeIndex(int index)
    {
        mazeIndex = index;
    }

    public override void Initialize()
    {
        base.Initialize();

        Debug.Log($"{name} -> Initialize() ejecutado. MazeIndex (inicial) = {mazeIndex}");

        // 1) Loguear componentes de rayos (RayPerceptionSensorComponent3D)
        DebugRayComponents();

        // 2) Loguear los sensores REALES registrados en el Agent (Vector + Ray, etc.)
        DebugAgentSensors();
    }

    private void Start()
    {
        behaviorParameters = GetComponent<BehaviorParameters>();

        // En Start el agente YA fue posicionado por MazeGenerator
        initialPosition = transform.position;
        hasInitialPosition = true;

        Debug.Log($"{name} -> Start() ejecutado. MazeIndex = {mazeIndex}");
    }

    public override void OnEpisodeBegin()
    {
        if (hasInitialPosition)
        {
            transform.position = initialPosition;
            transform.rotation = Quaternion.identity;
        }

        // Reset anti-idle y anti-loop
        stepsSinceMove = 0;
        lastPosition = transform.position;
        stepsSinceProgress = 0;

        // Distancia inicial en campo de distancias A*
        if (MazeGenerator.Instance != null &&
            MazeGenerator.Instance.HasAnyDistanceField &&
            mazeIndex >= 0)
        {
            previousDistance =
                MazeGenerator.Instance.GetDistanceFromWorldPos(transform.position, mazeIndex);
        }
        else
        {
            previousDistance = int.MaxValue;
        }

        // Cachear el camino A* de este laberinto en coordenadas de mundo
        cachedPathWorldPositions = null;
        hasPathCache = false;
        if (MazeGenerator.Instance != null && mazeIndex >= 0)
        {
            cachedPathWorldPositions = MazeGenerator.Instance.GetPathForMaze(mazeIndex);
            if (cachedPathWorldPositions != null && cachedPathWorldPositions.Count > 0)
            {
                hasPathCache = true;
            }

            // Floors excluyendo inicio/fin/path
            mazeFloorRenderers = MazeGenerator.Instance.GetMazeFloorRenderers(mazeIndex);
            Debug.Log($"{name} -> OnEpisodeBegin(): Floors en lista = {mazeFloorRenderers.Count}");
        }

        if (agentRenderer != null && neutral != null)
        {
            agentRenderer.material = neutral;
        }

        if (neutral != null && mazeFloorRenderers != null)
        {
            foreach (var r in mazeFloorRenderers)
            {
                if (r != null)
                    r.material = neutral;
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 agentPos = transform.position;

        Vector3 goalPos = agentPos;
        if (MazeGenerator.Instance != null && mazeIndex >= 0)
        {
            goalPos = MazeGenerator.Instance.GetGoalWorldPosForMaze(mazeIndex);
        }

        // 9 floats manuales básicos
        sensor.AddObservation(agentPos);           // 3
        sensor.AddObservation(goalPos);            // 3
        sensor.AddObservation(goalPos - agentPos); // 3

        // Observación extra: distancia discreta al objetivo (campo de distancias A*)
        if (includeDistanceObservation &&
            MazeGenerator.Instance != null &&
            MazeGenerator.Instance.HasAnyDistanceField &&
            mazeIndex >= 0)
        {
            int d = MazeGenerator.Instance.GetDistanceFromWorldPos(agentPos, mazeIndex);
            float normDist = (d == int.MaxValue)
                ? 1f
                : Mathf.Clamp01(d / Mathf.Max(1f, distanceNormalizationFactor));
            sensor.AddObservation(normDist); // 1 float extra
        }

        if (!loggedVectorSize)
        {
            Debug.Log($"{name} -> VectorSensor size (sólo observaciones manuales) = {sensor.ObservationSize()}");
            loggedVectorSize = true;
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveInput;
        float turnInput;

        // Usar teclado sólo si el Behavior está realmente en heurístico
        if (behaviorParameters != null && behaviorParameters.IsInHeuristicMode())
        {
            moveInput = Input.GetAxis("Vertical");
            turnInput = Input.GetAxis("Horizontal");
        }
        else
        {
            moveInput = actions.ContinuousActions[0];
            turnInput = actions.ContinuousActions[1];
        }

        // Movimiento
        Vector3 movement = transform.forward * moveInput * moveSpeed * Time.deltaTime;
        transform.position += movement;

        float turnAmount = turnInput * turnSpeed * Time.deltaTime;
        transform.Rotate(0f, turnAmount, 0f);

        // Penalización por tiempo
        AddReward(timePenaltyPerStep);

        // Anti-idle: penalizar si se queda casi en el mismo sitio muchos steps
        if (useAntiIdle)
        {
            float movedDist = Vector3.Distance(transform.position, lastPosition);
            if (movedDist < minMoveDistance)
            {
                stepsSinceMove++;
            }
            else
            {
                stepsSinceMove = 0;
                lastPosition = transform.position;
            }

            if (stepsSinceMove >= maxStepsWithoutMoving)
            {
                AddReward(idlePenalty);
                EndEpisode();
                return;
            }
        }

        // Shaping con distancia A* (campo de distancias desde cada celda al goal)
        if (useDistanceShaping &&
            MazeGenerator.Instance != null &&
            MazeGenerator.Instance.HasAnyDistanceField &&
            mazeIndex >= 0)
        {
            int currentDistance =
                MazeGenerator.Instance.GetDistanceFromWorldPos(transform.position, mazeIndex);

            bool improved = false;
            if (currentDistance != int.MaxValue && previousDistance != int.MaxValue)
            {
                if (currentDistance < previousDistance)
                {
                    AddReward(rewardTowardsGoal);
                    improved = true;
                }
                else if (currentDistance > previousDistance)
                {
                    AddReward(penaltyAwayFromGoal);
                }
            }

            // Anti-loop: si no mejora la distancia durante varios pasos, penaliza y termina
            if (useAntiLoop)
            {
                if (improved)
                {
                    stepsSinceProgress = 0;
                }
                else
                {
                    stepsSinceProgress++;
                    if (stepsSinceProgress >= maxStepsWithoutProgress)
                    {
                        AddReward(noProgressPenalty);
                        EndEpisode();
                        return;
                    }
                }
            }

            previousDistance = currentDistance;
        }

        // Reward extra por mantenerse cerca del camino A* de este laberinto
        if (usePathShaping && hasPathCache)
        {
            Vector3 pos = transform.position;
            float minDist = float.MaxValue;

            for (int i = 0; i < cachedPathWorldPositions.Count; i++)
            {
                float d = Vector3.Distance(pos, cachedPathWorldPositions[i]);
                if (d < minDist)
                {
                    minDist = d;
                }
            }

            if (minDist <= pathNearThreshold)
            {
                AddReward(rewardOnPath);
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuous = actionsOut.ContinuousActions;
        continuous[0] = Input.GetAxis("Vertical");
        continuous[1] = Input.GetAxis("Horizontal");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<Goal>(out Goal goal))
        {
            AddReward(goalReward);

            // Refrescar lista de floors excluyendo inicio/fin/path
            if (MazeGenerator.Instance != null && mazeIndex >= 0)
            {
                mazeFloorRenderers = MazeGenerator.Instance.GetMazeFloorRenderers(mazeIndex);
            }

            if (agentRenderer != null && win != null)
                agentRenderer.material = win;

            if (mazeFloorRenderers != null && win != null)
            {
                foreach (var r in mazeFloorRenderers)
                {
                    if (r != null)
                        r.material = win;
                }
            }

            EndEpisode();
        }

        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            AddReward(wallPenalty);

            // Refrescar lista de floors excluyendo inicio/fin/path
            if (MazeGenerator.Instance != null && mazeIndex >= 0)
            {
                mazeFloorRenderers = MazeGenerator.Instance.GetMazeFloorRenderers(mazeIndex);
            }

            if (agentRenderer != null && lose != null)
                agentRenderer.material = lose;

            if (mazeFloorRenderers != null && lose != null)
            {
                foreach (var r in mazeFloorRenderers)
                {
                    if (r != null)
                        r.material = lose;
                }
            }

            EndEpisode();
        }
    }

    // ================== DEBUG EXTRA ==================

    /// <summary>
    /// Ver cuántos RayPerceptionSensorComponent3D hay como hijos y sus offsets.
    /// </summary>
    private void DebugRayComponents()
    {
        if (loggedRayComponents) return;

        var raySensors = GetComponentsInChildren<RayPerceptionSensorComponent3D>();
        Debug.Log($"{name} -> RayPerceptionSensorComponent3D encontrados (en hijos): {raySensors.Length}");

        for (int i = 0; i < raySensors.Length; i++)
        {
            var rs = raySensors[i];
            Debug.Log(
                $"{name} -> RaySensorComponent[{i}] GameObject = {rs.gameObject.name} | " +
                $"StartVerticalOffset={rs.StartVerticalOffset}, EndVerticalOffset={rs.EndVerticalOffset}"
            );
        }

        loggedRayComponents = true;
    }

    /// <summary>
    /// Usar reflection para leer los sensores REALES registrados en el Agent (Vector + Ray, etc.).
    /// OJO: puede fallar si cambió la implementación interna de ML-Agents.
    /// </summary>
    private void DebugAgentSensors()
    {
        if (loggedAgentSensors) return;

        try
        {
            var field = typeof(Agent).GetField("m_Sensors",
                BindingFlags.Instance | BindingFlags.NonPublic);

            if (field == null)
            {
                Debug.LogWarning($"{name} -> No encontré el campo privado 'm_Sensors' en Agent (puede haber cambiado la versión de ML-Agents).");
                return;
            }

            var value = field.GetValue(this);
            if (value is List<ISensor> sensors)
            {
                Debug.Log($"{name} -> Total de sensores registrados en Agent = {sensors.Count}");

                for (int i = 0; i < sensors.Count; i++)
                {
                    var s = sensors[i];
                    var spec = s.GetObservationSpec();

                    int totalSize = 1;
                    if (spec.Shape.Length > 0)
                    {
                        for (int j = 0; j < spec.Shape.Length; j++)
                        {
                            totalSize *= spec.Shape[j];
                        }
                    }

                    string shapeStr;
                    if (spec.Shape.Length > 0)
                    {
                        string[] parts = new string[spec.Shape.Length];
                        for (int j = 0; j < spec.Shape.Length; j++)
                        {
                            parts[j] = spec.Shape[j].ToString();
                        }
                        shapeStr = string.Join("x", parts);
                    }
                    else
                    {
                        shapeStr = "[]";
                    }

                    Debug.Log(
                        $"{name} -> Sensor[{i}] Name={s.GetName()}, Shape={shapeStr}, TotalObs={totalSize}, Type={spec.ObservationType}"
                    );
                }
            }
            else
            {
                Debug.LogWarning($"{name} -> No pude castear m_Sensors a List<ISensor>.");
            }

            loggedAgentSensors = true;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"{name} -> Error al usar reflection para leer m_Sensors: {e.Message}");
        }
    }
}
