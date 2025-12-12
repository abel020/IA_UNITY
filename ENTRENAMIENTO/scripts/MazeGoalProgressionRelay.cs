using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class ProgressiveMazeGenerator : MonoBehaviour
{
    [Header("Prefab y dimensiones (modo clásico)")]
    public GameObject cellPrefab;      // Prefab que contiene el suelo y las 4 paredes.
    public int width = 5;              // Número de celdas en el ancho del laberinto
    public int height = 5;             // Número de celdas en la altura del laberinto
    public float cellSizeX = 5.0f;     // Tamaño de la celda en el eje X (ajustable según tu prefab)
    public float cellSizeZ = 5.0f;     // Tamaño de la celda en el eje Z (ajustable según tu prefab)

    [Header("Múltiples laberintos (modo clásico)")]
    public int mazeRows = 1;           // Número de laberintos en filas
    public int mazeColumns = 1;        // Número de laberintos en columnas
    public float mazePadding = 2.0f;   // Espacio entre laberintos

    [Header("Progresión escalonada 2x2→5x5")]
    public bool useProgressiveSet = false;
    public int mazesPerSize = 10;      // Cuántos laberintos generar por cada tamaño
    public int minProgressiveSize = 2; // Primer tamaño (ej. 2x2)
    public int maxProgressiveSize = 5; // Último tamaño (ej. 5x5)
    public float progressivePadding = 4.0f; // Espacio entre laberintos progresivos

    [Header("Materiales")]
    public Material startMaterial;     // Material para la celda de inicio
    public Material endMaterial;       // Material para la celda de fin
    public Material pathMaterial;      // Material opcional para pintar el camino A*
    public bool drawPathOnStart = false; // Si es true, pinta el camino A* al generar

    [Header("Personajes")]
    public GameObject playerPrefab;    // Prefab del personaje a instanciar en el inicio
    [Min(1)]
    public int playerInstances = 1;    // Cuántos prefabs de jugador instanciar por punto de inicio
    public float playerSpawnYOffset = 1.0f; // Altura de spawn respecto al suelo
    public GameObject followCameraPrefab; // Cámara a instanciar y parentar al agente
    public Vector3 followCameraLocalPosition = new Vector3(0f, 8f, -10f);
    public Vector3 followCameraLocalEuler = new Vector3(20f, 0f, 0f);

    [Header("Objetivo")]
    public GameObject goalPrefab;      // Prefab que se instancia en el final del laberinto
    public float goalSpawnYOffset = 1.0f;   // Altura de spawn respecto al suelo

    // Singleton para que los agentes puedan consultar el generador
    public static ProgressiveMazeGenerator Instance { get; private set; }

    // Lista de todos los laberintos generados
    private readonly List<MazeInstance> mazes = new();
    // Orden progresivo (cuando useProgressiveSet = true)
    private readonly List<MazeInstance> progressiveOrder = new();
    // Progreso de cada agente en la lista progressiveOrder
    private readonly Dictionary<Move_Laberynt_Agent, int> agentProgressIndex = new();
    // Agentes compartidos (solo se instancian en el primer laberinto progresivo)
    private readonly List<Move_Laberynt_Agent> sharedAgents = new();
    // Caminos A* y campos de distancias por laberinto
    private readonly Dictionary<MazeInstance, List<Cell>> mazePaths = new();
    private readonly Dictionary<MazeInstance, int[,]> mazeDistanceFields = new();

    private static readonly Vector2Int[] Directions =
    {
        new Vector2Int(1, 0),   // Este
        new Vector2Int(-1, 0),  // Oeste
        new Vector2Int(0, 1),   // Norte
        new Vector2Int(0, -1)   // Sur
    };

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning("MazeGenerator: ya existe otra instancia en la escena.");
        }
        Instance = this;
    }

    private void Start()
    {
        GenerateAllMazes();
    }

    // Clase que representa cada celda en el laberinto
    private class Cell
    {
        public readonly int X;
        public readonly int Y;
        public bool Visited;
        public GameObject[] Walls;   // Las paredes de la celda (Norte, Sur, Este, Oeste)
        public GameObject CellObject; // El objeto de la celda para aplicar materiales

        public Cell(GameObject[] wallObjs, GameObject cellObj, int x, int y)
        {
            Walls = wallObjs;
            CellObject = cellObj;
            X = x;
            Y = y;
            Visited = false;
        }
    }

    // Contenedor para cada laberinto generado
    private class MazeInstance
    {
        public readonly Cell[,] Grid;
        public readonly List<Cell> Frontier = new();
        public readonly Vector3 Origin;
        public readonly int Width;
        public readonly int Height;
        public readonly int Index;

        public Cell StartCell;
        public Cell EndCell;

        public MazeInstance(int width, int height, Vector3 origin, int index)
        {
            Width = width;
            Height = height;
            Grid = new Cell[width, height];
            Origin = origin;
            Index = index;
        }
    }

    // Nodo usado por A*
    private class AStarNode
    {
        public Cell Cell;
        public float G; // costo desde el inicio
        public float H; // heurística al objetivo
        public float F => G + H;
        public AStarNode Parent;

        public AStarNode(Cell cell)
        {
            Cell = cell;
        }
    }

    /// <summary>
    /// Componente interno que vive en cada Goal instanciado. Reenvía el trigger al generador
    /// para avanzar al siguiente laberinto de la secuencia progresiva.
    /// </summary>
    private class GoalRelay : MonoBehaviour
    {
        private void OnTriggerEnter(Collider other)
        {
            var agent = other.GetComponent<Move_Laberynt_Agent>();
            if (agent != null && ProgressiveMazeGenerator.Instance != null)
            {
                ProgressiveMazeGenerator.Instance.AdvanceAgentToNextMaze(agent);
            }
        }
    }

    // Genera todos los laberintos solicitados (modo clásico o progresivo)
    private void GenerateAllMazes()
    {
        mazes.Clear();
        progressiveOrder.Clear();
        agentProgressIndex.Clear();
        mazePaths.Clear();
        mazeDistanceFields.Clear();

        if (useProgressiveSet)
        {
            float offsetZ = 0f;
            for (int size = minProgressiveSize; size <= maxProgressiveSize; size++)
            {
                for (int i = 0; i < mazesPerSize; i++)
                {
                    float mazeTotalDepth = size * cellSizeZ + progressivePadding;
                    Vector3 origin = new Vector3(0f, 0f, offsetZ);

                    int mazeIndex = mazes.Count;
                    MazeInstance instance = new MazeInstance(size, size, origin, mazeIndex);
                    mazes.Add(instance);
                    progressiveOrder.Add(instance);

                    BuildMaze(instance);
                    StartCoroutine(GenerateMazeUsingPrim(instance));

                    offsetZ += mazeTotalDepth;
                }
            }
        }
        else
        {
            float mazeTotalWidth = width * cellSizeX + mazePadding;
            float mazeTotalHeight = height * cellSizeZ + mazePadding;

            for (int row = 0; row < mazeRows; row++)
            {
                for (int col = 0; col < mazeColumns; col++)
                {
                    Vector3 origin = new Vector3(col * mazeTotalWidth, 0f, row * mazeTotalHeight);
                    int mazeIndex = mazes.Count;
                    MazeInstance instance = new MazeInstance(width, height, origin, mazeIndex);
                    mazes.Add(instance);
                    BuildMaze(instance);
                    StartCoroutine(GenerateMazeUsingPrim(instance));
                }
            }
        }
    }

    // Construye la malla de celdas de un laberinto individual
    private void BuildMaze(MazeInstance instance)
    {
        for (int x = 0; x < instance.Width; x++)
        {
            for (int y = 0; y < instance.Height; y++)
            {
                Vector3 cellPosition = instance.Origin + new Vector3(x * cellSizeX, 0, y * cellSizeZ);
                GameObject cellObject = Instantiate(cellPrefab, cellPosition, Quaternion.identity, transform);
                GameObject[] walls = GetWalls(cellObject);

                instance.Grid[x, y] = new Cell(walls, cellObject, x, y);

                if (x == 0 && y == 0)
                {
                    SetMaterial(cellObject, startMaterial);
                    instance.StartCell = instance.Grid[x, y];
                }
                else if (x == instance.Width - 1 && y == instance.Height - 1)
                {
                    SetMaterial(cellObject, endMaterial);
                    instance.EndCell = instance.Grid[x, y];
                }
            }
        }

        // Asegura un campo de distancias placeholder para que los agentes siempre
        // puedan añadir la observación escalar de distancia sin producir padding.
        EnsurePlaceholderDistanceField(instance);

        // En modo progresivo solo se instancian agentes en el primer laberinto generado;
        // el resto recicla los mismos agentes al teletransportarlos.
        if (useProgressiveSet)
        {
            if (progressiveOrder.Count > 0 && instance == progressiveOrder[0])
            {
                int toSpawn = Mathf.Max(0, playerInstances - sharedAgents.Count);
                for (int i = 0; i < toSpawn; i++)
                {
                    SpawnPlayer(instance, instance.StartCell, trackProgressive: true);
                }
            }
        }
        else
        {
            for (int i = 0; i < playerInstances; i++)
            {
                SpawnPlayer(instance, instance.StartCell, trackProgressive: false);
            }
        }
        SpawnGoal(instance, instance.EndCell);
    }

    // Instancia un jugador en la celda inicial del laberinto
    private void SpawnPlayer(MazeInstance instance, Cell startCell, bool trackProgressive)
    {
        if (playerPrefab == null || startCell?.CellObject == null)
        {
            return;
        }

        Vector3 spawnPosition = startCell.CellObject.transform.position;
        spawnPosition.y += playerSpawnYOffset;
        GameObject player = Instantiate(playerPrefab, spawnPosition, Quaternion.identity, transform);

        var agent = player.GetComponent<Move_Laberynt_Agent>();
        if (agent != null)
        {
            agent.SetMazeIndex(instance.Index);

            if (trackProgressive)
            {
                sharedAgents.Add(agent);
                agentProgressIndex[agent] = 0;
            }
        }

        // Instanciar y parentar la cámara de seguimiento si se configuró en el inspector
        if (followCameraPrefab != null && player.GetComponentInChildren<Camera>() == null)
        {
            GameObject cam = Instantiate(followCameraPrefab, player.transform);
            cam.transform.localPosition = followCameraLocalPosition;
            cam.transform.localRotation = Quaternion.Euler(followCameraLocalEuler);
        }
    }

    // Instancia un objeto objetivo en la celda final del laberinto
    private void SpawnGoal(MazeInstance instance, Cell endCell)
    {
        if (goalPrefab == null || endCell?.CellObject == null)
        {
            return;
        }

        Vector3 spawnPosition = endCell.CellObject.transform.position;
        spawnPosition.y += goalSpawnYOffset;
        GameObject goal = Instantiate(goalPrefab, spawnPosition, Quaternion.identity, transform);

        // Añadimos el relay interno para avanzar al siguiente laberinto cuando un agente llegue.
        if (goal.GetComponent<GoalRelay>() == null)
        {
            goal.AddComponent<GoalRelay>();
        }
    }

    // Obtiene las paredes del prefab de la celda
    private GameObject[] GetWalls(GameObject cell)
    {
        GameObject[] walls = new GameObject[4];
        Transform wallsTransform = cell.transform.Find("Walls");

        if (wallsTransform != null)
        {
            walls[0] = wallsTransform.Find("Wall_North") != null ? wallsTransform.Find("Wall_North").gameObject : null;
            walls[1] = wallsTransform.Find("Wall_South") != null ? wallsTransform.Find("Wall_South").gameObject : null;
            walls[2] = wallsTransform.Find("Wall_East") != null ? wallsTransform.Find("Wall_East").gameObject : null;
            walls[3] = wallsTransform.Find("Wall_West") != null ? wallsTransform.Find("Wall_West").gameObject : null;
        }
        else
        {
            Debug.LogError("El objeto 'Walls' no se encontró dentro de la celda.");
        }

        return walls;
    }

    // Asigna el material al "Renderer" del objeto 'Ground' dentro de la celda
    private void SetMaterial(GameObject cellObject, Material material)
    {
        Transform groundTransform = cellObject.transform.Find("Ground");

        if (groundTransform != null)
        {
            Renderer renderer = groundTransform.GetComponent<Renderer>();
            if (renderer != null && material != null)
            {
                renderer.material = material;
            }
        }
    }

    // Algoritmo de Prim para generar el laberinto sin diagonales y siempre conectado
    private IEnumerator GenerateMazeUsingPrim(MazeInstance instance)
    {
        instance.Frontier.Clear();

        Cell startCell = instance.Grid[0, 0];
        startCell.Visited = true;

        AddNeighborsToFrontier(startCell, instance);

        while (instance.Frontier.Count > 0)
        {
            Cell currentCell = instance.Frontier[Random.Range(0, instance.Frontier.Count)];

            bool connected = ConnectToRandomVisitedNeighbor(currentCell, instance);

            if (connected)
            {
                currentCell.Visited = true;
                AddNeighborsToFrontier(currentCell, instance);
            }

            instance.Frontier.Remove(currentCell);

            yield return null;
        }

        // --- Al terminar de tallar el laberinto, calculamos camino A* y campo de distancias ---
        List<Cell> path = null;
        if (instance.StartCell != null && instance.EndCell != null)
        {
            path = FindPathAStar(instance, instance.StartCell, instance.EndCell);
            if (path != null && path.Count > 0)
            {
                mazePaths[instance] = path;

                if (drawPathOnStart && pathMaterial != null)
                {
                    HighlightPath(path, instance.StartCell, instance.EndCell);
                }
            }
        }

        ComputeDistanceField(instance);
    }

    // Función para añadir las celdas vecinas no visitadas a la frontera
    private void AddNeighborsToFrontier(Cell cell, MazeInstance instance)
    {
        foreach (Vector2Int direction in Directions)
        {
            int nx = cell.X + direction.x;
            int ny = cell.Y + direction.y;

            if (!IsInsideBounds(instance, nx, ny))
            {
                continue;
            }

            Cell neighbor = instance.Grid[nx, ny];
            if (!neighbor.Visited && !instance.Frontier.Contains(neighbor))
            {
                instance.Frontier.Add(neighbor);
            }
        }
    }

    // Conecta la celda actual con un vecino ya visitado, derribando la pared compartida
    private bool ConnectToRandomVisitedNeighbor(Cell currentCell, MazeInstance instance)
    {
        List<Cell> visitedNeighbors = new();

        foreach (Vector2Int direction in Directions)
        {
            int nx = currentCell.X + direction.x;
            int ny = currentCell.Y + direction.y;
            if (!IsInsideBounds(instance, nx, ny))
            {
                continue;
            }

            Cell neighbor = instance.Grid[nx, ny];
            if (neighbor.Visited)
            {
                visitedNeighbors.Add(neighbor);
            }
        }

        if (visitedNeighbors.Count == 0)
        {
            return false; // No hay vecinos visitados
        }

        Cell chosenNeighbor = visitedNeighbors[Random.Range(0, visitedNeighbors.Count)];
        RemoveWallBetweenCells(currentCell, chosenNeighbor);
        return true;
    }

    // Función para eliminar la pared entre dos celdas adyacentes
    private void RemoveWallBetweenCells(Cell currentCell, Cell neighborCell)
    {
        int dx = neighborCell.X - currentCell.X;
        int dy = neighborCell.Y - currentCell.Y;

        if (dx == 1)
        {
            if (currentCell.Walls[2] != null) Destroy(currentCell.Walls[2]);
            if (neighborCell.Walls[3] != null) Destroy(neighborCell.Walls[3]);
        }
        else if (dx == -1)
        {
            if (currentCell.Walls[3] != null) Destroy(currentCell.Walls[3]);
            if (neighborCell.Walls[2] != null) Destroy(neighborCell.Walls[2]);
        }
        else if (dy == 1)
        {
            if (currentCell.Walls[0] != null) Destroy(currentCell.Walls[0]);
            if (neighborCell.Walls[1] != null) Destroy(neighborCell.Walls[1]);
        }
        else if (dy == -1)
        {
            if (currentCell.Walls[1] != null) Destroy(currentCell.Walls[1]);
            if (neighborCell.Walls[0] != null) Destroy(neighborCell.Walls[0]);
        }
    }

    private bool IsInsideBounds(MazeInstance instance, int x, int y)
    {
        return x >= 0 && y >= 0 && x < instance.Width && y < instance.Height;
    }

    // Prepara un campo de distancias vacío (int.MaxValue) para que los agentes
    // puedan usar la observación de distancia desde el primer frame.
    private void EnsurePlaceholderDistanceField(MazeInstance instance)
    {
        if (instance == null)
        {
            return;
        }

        int[,] field = new int[instance.Width, instance.Height];
        for (int x = 0; x < instance.Width; x++)
        {
            for (int y = 0; y < instance.Height; y++)
            {
                field[x, y] = int.MaxValue;
            }
        }

        mazeDistanceFields[instance] = field;
    }

    // ===================== A* Y CAMPO DE DISTANCIAS =====================

    private float Heuristic(Cell a, Cell b)
    {
        return Mathf.Abs(a.X - b.X) + Mathf.Abs(a.Y - b.Y); // Manhattan
    }

    private List<Cell> FindPathAStar(MazeInstance instance, Cell start, Cell goal)
    {
        var open = new List<AStarNode>();
        var closed = new HashSet<Cell>();

        var startNode = new AStarNode(start) { G = 0f, H = Heuristic(start, goal) };
        open.Add(startNode);

        while (open.Count > 0)
        {
            AStarNode current = open[0];
            for (int i = 1; i < open.Count; i++)
            {
                if (open[i].F < current.F)
                {
                    current = open[i];
                }
            }

            if (current.Cell == goal)
            {
                return ReconstructPath(current);
            }

            open.Remove(current);
            closed.Add(current.Cell);

            foreach (Vector2Int dir in Directions)
            {
                int nx = current.Cell.X + dir.x;
                int ny = current.Cell.Y + dir.y;
                if (!IsInsideBounds(instance, nx, ny)) continue;

                Cell neighbor = instance.Grid[nx, ny];
                if (!AreCellsConnected(current.Cell, neighbor) || closed.Contains(neighbor))
                    continue;

                float tentativeG = current.G + 1f;
                AStarNode neighborNode = open.Find(n => n.Cell == neighbor);
                if (neighborNode == null)
                {
                    neighborNode = new AStarNode(neighbor)
                    {
                        G = tentativeG,
                        H = Heuristic(neighbor, goal),
                        Parent = current
                    };
                    open.Add(neighborNode);
                }
                else if (tentativeG < neighborNode.G)
                {
                    neighborNode.G = tentativeG;
                    neighborNode.Parent = current;
                }
            }
        }

        return null;
    }

    private bool AreCellsConnected(Cell a, Cell b)
    {
        int dx = b.X - a.X;
        int dy = b.Y - a.Y;

        if (dx == 1) return a.Walls[2] == null && b.Walls[3] == null;
        if (dx == -1) return a.Walls[3] == null && b.Walls[2] == null;
        if (dy == 1) return a.Walls[0] == null && b.Walls[1] == null;
        if (dy == -1) return a.Walls[1] == null && b.Walls[0] == null;
        return false;
    }

    private List<Cell> ReconstructPath(AStarNode endNode)
    {
        var path = new List<Cell>();
        AStarNode current = endNode;
        while (current != null)
        {
            path.Add(current.Cell);
            current = current.Parent;
        }
        path.Reverse();
        return path;
    }

    private void HighlightPath(List<Cell> path, Cell start, Cell end)
    {
        if (path == null || path.Count == 0 || pathMaterial == null) return;

        foreach (Cell cell in path)
        {
            if (cell == start || cell == end || cell.CellObject == null) continue;
            Transform ground = cell.CellObject.transform.Find("Ground");
            if (ground == null) continue;
            Renderer renderer = ground.GetComponent<Renderer>();
            if (renderer == null) continue;
            renderer.material = pathMaterial;
        }
    }

    private void ComputeDistanceField(MazeInstance instance)
    {
        if (instance.EndCell == null) return;

        int[,] field = new int[instance.Width, instance.Height];
        for (int x = 0; x < instance.Width; x++)
        {
            for (int y = 0; y < instance.Height; y++)
            {
                field[x, y] = int.MaxValue;
            }
        }

        var queue = new Queue<Cell>();
        field[instance.EndCell.X, instance.EndCell.Y] = 0;
        queue.Enqueue(instance.EndCell);

        while (queue.Count > 0)
        {
            Cell current = queue.Dequeue();
            int currentDist = field[current.X, current.Y];

            foreach (Vector2Int dir in Directions)
            {
                int nx = current.X + dir.x;
                int ny = current.Y + dir.y;
                if (!IsInsideBounds(instance, nx, ny)) continue;

                Cell neighbor = instance.Grid[nx, ny];
                if (!AreCellsConnected(current, neighbor)) continue;

                int newDist = currentDist + 1;
                if (newDist < field[nx, ny])
                {
                    field[nx, ny] = newDist;
                    queue.Enqueue(neighbor);
                }
            }
        }

        mazeDistanceFields[instance] = field;
    }

    // ===================== API PÚBLICA PARA AGENTES =====================

    public bool HasAnyDistanceField => mazeDistanceFields.Count > 0;

    /// <summary>
    /// Devuelve la lista de renderers de suelo (Ground) de un laberinto,
    /// excluyendo inicio, fin y las celdas del camino A*.
    /// </summary>
    public List<MeshRenderer> GetMazeFloorRenderers(int mazeIndex)
    {
        var result = new List<MeshRenderer>();

        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
        {
            return result;
        }

        MazeInstance instance = mazes[mazeIndex];
        mazePaths.TryGetValue(instance, out List<Cell> pathCells);
        HashSet<Cell> pathSet = pathCells != null ? new HashSet<Cell>(pathCells) : null;

        for (int x = 0; x < instance.Width; x++)
        {
            for (int y = 0; y < instance.Height; y++)
            {
                Cell cell = instance.Grid[x, y];
                if (cell == null || cell.CellObject == null)
                {
                    continue;
                }

                if (cell == instance.StartCell || cell == instance.EndCell)
                {
                    continue;
                }

                if (pathSet != null && pathSet.Contains(cell))
                {
                    continue;
                }

                Transform ground = cell.CellObject.transform.Find("Ground");
                if (ground == null)
                {
                    continue;
                }

                MeshRenderer renderer = ground.GetComponent<MeshRenderer>();
                if (renderer != null)
                {
                    result.Add(renderer);
                }
            }
        }

        return result;
    }

    /// <summary>
    /// Devuelve la posición objetivo del laberinto por índice.
    /// </summary>
    public Vector3 GetGoalWorldPosForMaze(int mazeIndex)
    {
        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
            return Vector3.zero;

        MazeInstance instance = mazes[mazeIndex];
        if (instance.EndCell?.CellObject == null)
            return Vector3.zero;

        return instance.EndCell.CellObject.transform.position;
    }

    /// <summary>
    /// Devuelve la posición de inicio del laberinto por índice (usada para teletransportar agentes).
    /// </summary>
    public Vector3 GetStartPositionForMaze(int mazeIndex)
    {
        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
            return Vector3.zero;

        MazeInstance instance = mazes[mazeIndex];
        if (instance.StartCell?.CellObject == null)
            return Vector3.zero;

        Vector3 pos = instance.StartCell.CellObject.transform.position;
        pos.y += playerSpawnYOffset;
        return pos;
    }

    /// <summary>
    /// Convierte worldPos a coordenadas de celda dentro de un MazeInstance.
    /// </summary>
    private bool WorldToCell(MazeInstance instance, Vector3 worldPos, out int gx, out int gy)
    {
        float localX = (worldPos.x - instance.Origin.x) / cellSizeX;
        float localZ = (worldPos.z - instance.Origin.z) / cellSizeZ;

        gx = Mathf.RoundToInt(localX);
        gy = Mathf.RoundToInt(localZ);

        return IsInsideBounds(instance, gx, gy);
    }

    /// <summary>
    /// Distancia desde una posición de mundo al goal del laberinto indicado (usada por el agente).
    /// </summary>
    public int GetDistanceFromWorldPos(Vector3 worldPos, int mazeIndex)
    {
        if (mazeIndex < 0 || mazeIndex >= mazes.Count) return int.MaxValue;
        MazeInstance instance = mazes[mazeIndex];
        if (!WorldToCell(instance, worldPos, out int gx, out int gy)) return int.MaxValue;
        if (!mazeDistanceFields.TryGetValue(instance, out int[,] field)) return int.MaxValue;
        return field[gx, gy];
    }

    /// <summary>
    /// Camino A* del laberinto indicado en posiciones de mundo.
    /// </summary>
    public List<Vector3> GetPathForMaze(int mazeIndex)
    {
        if (mazeIndex < 0 || mazeIndex >= mazes.Count) return null;
        MazeInstance instance = mazes[mazeIndex];
        if (!mazePaths.TryGetValue(instance, out List<Cell> pathCells) || pathCells == null) return null;

        var result = new List<Vector3>(pathCells.Count);
        foreach (Cell cell in pathCells)
        {
            if (cell?.CellObject != null)
            {
                result.Add(cell.CellObject.transform.position);
            }
        }
        return result;
    }

    /// <summary>
    /// Avanza al siguiente laberinto de la secuencia (10 mazes por tamaño de 2 a 5).
    /// Teletransporta al agente al inicio del siguiente laberinto.
    /// </summary>
    public void AdvanceAgentToNextMaze(Move_Laberynt_Agent agent)
    {
        if (agent == null)
        {
            return;
        }

        if (!useProgressiveSet || progressiveOrder.Count == 0)
        {
            Debug.LogWarning("AdvanceAgentToNextMaze: el modo progresivo no está activo o no hay laberintos generados.");
            return;
        }

        if (!agentProgressIndex.TryGetValue(agent, out int idx))
        {
            idx = 0;
        }

        idx++;

        if (idx >= progressiveOrder.Count)
        {
            idx = 0; // reiniciamos el ciclo tras completar 2x2→5x5 (10 cada uno)
            Debug.Log("Agent completó todos los laberintos de la secuencia, reiniciando en el primero.");
        }

        agentProgressIndex[agent] = idx;
        MazeInstance nextMaze = progressiveOrder[idx];

        Vector3 nextPos = GetStartPositionForMaze(nextMaze.Index);
        TeleportAndRetargetAgent(agent, nextMaze, nextPos);
        StartCoroutine(TeleportDeferred(agent, nextMaze, nextPos));
    }

    private void TeleportAndRetargetAgent(Move_Laberynt_Agent agent, MazeInstance targetMaze, Vector3 targetPos)
    {
        if (agent == null || targetMaze == null)
        {
            return;
        }

        agent.transform.position = targetPos;
        agent.transform.rotation = Quaternion.identity;
        agent.SetMazeIndex(targetMaze.Index);

        TrySetInitialPositionFields(agent, targetPos);
    }

    private IEnumerator TeleportDeferred(Move_Laberynt_Agent agent, MazeInstance targetMaze, Vector3 targetPos)
    {
        yield return null; // Espera un frame para sobreescribir cualquier reset de OnEpisodeBegin()
        TeleportAndRetargetAgent(agent, targetMaze, targetPos);
    }

    private void TrySetInitialPositionFields(Move_Laberynt_Agent agent, Vector3 pos)
    {
        const BindingFlags Flags = BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public;

        var type = agent.GetType();
        var initialPosField = type.GetField("initialPosition", Flags);
        var hasInitField = type.GetField("hasInitialPosition", Flags);

        if (initialPosField != null)
        {
            initialPosField.SetValue(agent, pos);
        }

        if (hasInitField != null)
        {
            hasInitField.SetValue(agent, true);
        }
    }
}