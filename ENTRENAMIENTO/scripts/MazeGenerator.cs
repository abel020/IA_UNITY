using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class DebugMazeInfo
{
    public int mazeIndex;
    public Vector3 origin;
    public Vector2Int startCell;
    public Vector2Int endCell;
    public int pathLength;
}

public class MazeGenerator : MonoBehaviour
{
    [Header("Prefab y dimensiones")]
    public GameObject cellPrefab;      // Prefab que contiene el suelo y las 4 paredes.
    public int width = 5;              // Número de celdas en el ancho del laberinto
    public int height = 5;             // Número de celdas en la altura del laberinto
    public float cellSizeX = 5.0f;     // Tamaño de la celda en el eje X (ajustable según tu prefab)
    public float cellSizeZ = 5.0f;     // Tamaño de la celda en el eje Z (ajustable según tu prefab)

    [Header("Múltiples laberintos")]
    public int mazeRows = 1;           // Número de laberintos en filas
    public int mazeColumns = 1;        // Número de laberintos en columnas
    public float mazePadding = 2.0f;   // Espacio entre laberintos

    [Header("Materiales")]
    public Material startMaterial;     // Material para la celda de inicio
    public Material endMaterial;       // Material para la celda de fin

    [Header("Personajes")]
    public GameObject playerPrefab;    // Prefab del personaje a instanciar en el inicio
    public float playerSpawnYOffset = 1.0f; // Altura de spawn respecto al suelo

    [Header("Objetivo")]
    public GameObject goalPrefab;      // Prefab que se instancia en el final del laberinto
    public float goalSpawnYOffset = 1.0f;   // Altura de spawn respecto al suelo

    [Header("Pathfinding (A*)")]
    public bool drawPathOnStart = false;   // Si es true, dibuja el camino de A* al terminar de generar
    public Material pathMaterial;          // Material para pintar el camino

    [Header("DEBUG")]
    [SerializeField] private List<DebugMazeInfo> debugMazes = new();

    // Singleton para que el agente pueda acceder fácil
    public static MazeGenerator Instance { get; private set; }

    // Lista de todos los laberintos generados
    private readonly List<MazeInstance> mazes = new();

    // Por cada laberinto, guardamos su camino A* y su campo de distancias
    private readonly Dictionary<MazeInstance, List<Cell>> mazePaths = new();
    private readonly Dictionary<MazeInstance, int[,]> mazeDistanceFields = new();

    // Laberinto "principal" para RL (por ejemplo el primero)
    private MazeInstance mainInstance;
    // Campo de distancias activo (del laberinto principal) – solo para API antigua
    private int[,] distanceField;
    // Último path solo para debug
    private List<Cell> lastPath;

    private static readonly Vector2Int[] Directions =
    {
        new Vector2Int(1, 0),   // Este
        new Vector2Int(-1, 0),  // Oeste
        new Vector2Int(0, 1),   // Norte
        new Vector2Int(0, -1)   // Sur
    };

    // ===================== CICLO DE VIDA =====================

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning("MazeGenerator: hay más de una instancia en la escena.");
        }
        Instance = this;
    }

    private void Start()
    {
        GenerateAllMazes();
    }

    // ===================== CLASES INTERNAS =====================

    // Clase que representa cada celda en el laberinto
    private class Cell
    {
        public readonly int X;
        public readonly int Y;
        public bool Visited;
        public GameObject[] Walls;    // Las paredes de la celda (Norte, Sur, Este, Oeste)
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
        public readonly int Index;

        // Inicio y fin de este laberinto
        public Cell StartCell;
        public Cell EndCell;

        public MazeInstance(int width, int height, Vector3 origin, int index)
        {
            Grid = new Cell[width, height];
            Origin = origin;
            Index = index;
        }
    }

    // Nodo para A*
    private class AStarNode
    {
        public Cell Cell;
        public float G; // costo desde el inicio
        public float H; // heurística (estimación al objetivo)
        public float F => G + H;
        public AStarNode Parent;

        public AStarNode(Cell cell)
        {
            Cell = cell;
        }
    }

    // ===================== GENERACIÓN DE LABERINTOS =====================

    private void GenerateAllMazes()
    {
        mazes.Clear();
        mazePaths.Clear();
        mazeDistanceFields.Clear();
        debugMazes.Clear();
        mainInstance = null;
        distanceField = null;
        lastPath = null;

        float mazeTotalWidth = width * cellSizeX + mazePadding;
        float mazeTotalHeight = height * cellSizeZ + mazePadding;

        for (int row = 0; row < mazeRows; row++)
        {
            for (int col = 0; col < mazeColumns; col++)
            {
                Vector3 origin = new Vector3(col * mazeTotalWidth, 0f, row * mazeTotalHeight);

                // El índice del laberinto será su posición en la lista
                int mazeIndex = mazes.Count;
                MazeInstance instance = new MazeInstance(width, height, origin, mazeIndex);
                mazes.Add(instance);

                // El primero que se crea es el "principal" (por compatibilidad)
                if (mainInstance == null)
                {
                    mainInstance = instance;
                }

                BuildMaze(instance);                     // Instancia todas las celdas
                ChooseStartAndEnd(instance, out Cell startCell, out Cell endCell); // Elige extremos
                instance.StartCell = startCell;          // Guardamos para A*
                instance.EndCell = endCell;              // Guardamos para A*
                ApplyStartEndVisuals(startCell, endCell); // Pinta materiales
                SpawnPlayer(instance, startCell);        // Instancia jugador (Agent) en ESTE laberinto
                SpawnGoal(instance, endCell);            // Instancia objetivo en ESTE laberinto

                StartCoroutine(GenerateMazeUsingPrim(instance)); // Carva el laberinto
            }
        }
    }

    // Construye la malla de celdas de un laberinto individual
    private void BuildMaze(MazeInstance instance)
    {
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                Vector3 cellPosition = instance.Origin + new Vector3(x * cellSizeX, 0, y * cellSizeZ);
                GameObject cellObject = Instantiate(
                    cellPrefab,
                    cellPosition,
                    Quaternion.identity,
                    transform
                );
                GameObject[] walls = GetWalls(cellObject);

                instance.Grid[x, y] = new Cell(walls, cellObject, x, y);
            }
        }
    }

    // ---------- Elección de inicio y fin extremos ----------

    /// <summary>
    /// Elige dos celdas de los bordes del laberinto como inicio y fin,
    /// intentando que estén lo más alejadas posible (extremo a extremo).
    /// </summary>
    private void ChooseStartAndEnd(MazeInstance instance, out Cell startCell, out Cell endCell)
    {
        if (width < 2 && height < 2)
        {
            startCell = instance.Grid[0, 0];
            endCell = instance.Grid[0, 0];
            return;
        }

        bool horizontal = Random.value < 0.5f; // true: izquierda-derecha, false: abajo-arriba

        int startX, startY, endX, endY;

        if (horizontal)
        {
            // Inicio en el borde izquierdo (x = 0), fin en el borde derecho (x = width - 1)
            startX = 0;
            endX = width - 1;

            startY = Random.Range(0, height);
            endY = Random.Range(0, height);

            int intentos = 0;
            while (Mathf.Abs(startY - endY) < height / 2 && intentos < 10 && height > 1)
            {
                endY = Random.Range(0, height);
                intentos++;
            }
        }
        else
        {
            // Inicio en el borde inferior (y = 0), fin en el borde superior (y = height - 1)
            startY = 0;
            endY = height - 1;

            startX = Random.Range(0, width);
            endX = Random.Range(0, width);

            int intentos = 0;
            while (Mathf.Abs(startX - endX) < width / 2 && intentos < 10 && width > 1)
            {
                endX = Random.Range(0, width);
                intentos++;
            }
        }

        startCell = instance.Grid[startX, startY];
        endCell = instance.Grid[endX, endY];
    }

    private void ApplyStartEndVisuals(Cell startCell, Cell endCell)
    {
        if (startCell != null && startMaterial != null)
        {
            SetMaterial(startCell.CellObject, startMaterial);
        }

        if (endCell != null && endMaterial != null)
        {
            SetMaterial(endCell.CellObject, endMaterial);
        }
    }

    // ---------- Spawn de jugador y objetivo ----------

    // Instancia un jugador en la celda de inicio de ESTE laberinto
    private void SpawnPlayer(MazeInstance instance, Cell startCell)
    {
        if (playerPrefab == null || startCell == null)
        {
            return;
        }

        Vector3 pos = startCell.CellObject.transform.position;
        pos.y += playerSpawnYOffset;

        GameObject playerObj = Instantiate(playerPrefab, pos, Quaternion.identity, transform);

        // IMPORTANTÍSIMO: decirle al Agent de qué laberinto es
        var agent = playerObj.GetComponent<Move_Laberynt_Agent>();
        if (agent != null)
        {
            agent.SetMazeIndex(instance.Index);
        }

        // Para debug:
        Debug.Log($"SpawnPlayer -> Maze {instance.Index}, StartCell ({startCell.X},{startCell.Y}), pos {pos}");
    }

    // Instancia un objeto objetivo en la celda de fin de ESTE laberinto
    private void SpawnGoal(MazeInstance instance, Cell endCell)
    {
        if (goalPrefab == null || endCell == null)
        {
            return;
        }

        Vector3 pos = endCell.CellObject.transform.position;
        pos.y += goalSpawnYOffset;
        Instantiate(goalPrefab, pos, Quaternion.identity, transform);
    }

    // ---------- Utilidades de celdas ----------

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

    private bool IsInsideBounds(int x, int y)
    {
        return x >= 0 && y >= 0 && x < width && y < height;
    }

    // ===================== ALGORITMO DE PRIM =====================

    private IEnumerator GenerateMazeUsingPrim(MazeInstance instance)
    {
        instance.Frontier.Clear();

        Cell startCell = instance.Grid[0, 0]; // semilla de generación
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

        // ---------- AQUÍ EL LABERINTO YA ESTÁ GENERADO ----------

        // 1) A*: calculamos SIEMPRE el camino de ESTE laberinto (para RL),
        // y opcionalmente lo pintamos si drawPathOnStart está activo.
        List<Cell> path = null;

        if (instance.StartCell != null && instance.EndCell != null)
        {
            path = FindPathAStar(instance, instance.StartCell, instance.EndCell);

            if (path != null && path.Count > 0)
            {
                // Guardamos el path de este laberinto para RL
                mazePaths[instance] = path;

                if (instance == mainInstance)
                {
                    lastPath = path;
                }

                // Pintar el camino A* solo si el usuario lo pide y tiene material
                if (drawPathOnStart && pathMaterial != null)
                {
                    HighlightPath(path, instance.StartCell, instance.EndCell);
                }
            }
        }

        // 2) Campo de distancias para ESTE laberinto (para RL o análisis)
        ComputeDistanceField(instance);

        // 3) DEBUG: info en el Inspector
        if (debugMazes == null)
            debugMazes = new List<DebugMazeInfo>();

        var info = new DebugMazeInfo
        {
            mazeIndex = instance.Index,
            origin = instance.Origin,
            startCell = instance.StartCell != null
                ? new Vector2Int(instance.StartCell.X, instance.StartCell.Y)
                : new Vector2Int(-1, -1),
            endCell = instance.EndCell != null
                ? new Vector2Int(instance.EndCell.X, instance.EndCell.Y)
                : new Vector2Int(-1, -1),
            pathLength = (path != null) ? path.Count : -1
        };

        debugMazes.Add(info);
    }

    private void AddNeighborsToFrontier(Cell cell, MazeInstance instance)
    {
        foreach (Vector2Int direction in Directions)
        {
            int nx = cell.X + direction.x;
            int ny = cell.Y + direction.y;

            if (!IsInsideBounds(nx, ny))
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

    private bool ConnectToRandomVisitedNeighbor(Cell currentCell, MazeInstance instance)
    {
        List<Cell> visitedNeighbors = new();

        foreach (Vector2Int direction in Directions)
        {
            int nx = currentCell.X + direction.x;
            int ny = currentCell.Y + direction.y;
            if (!IsInsideBounds(nx, ny))
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
            // Vecino a la derecha (Este)
            if (currentCell.Walls[2] != null) Destroy(currentCell.Walls[2]);
            if (neighborCell.Walls[3] != null) Destroy(neighborCell.Walls[3]);
        }
        else if (dx == -1)
        {
            // Vecino a la izquierda (Oeste)
            if (currentCell.Walls[3] != null) Destroy(currentCell.Walls[3]);
            if (neighborCell.Walls[2] != null) Destroy(neighborCell.Walls[2]);
        }
        else if (dy == 1)
        {
            // Vecino arriba (Norte)
            if (currentCell.Walls[0] != null) Destroy(currentCell.Walls[0]);
            if (neighborCell.Walls[1] != null) Destroy(neighborCell.Walls[1]);
        }
        else if (dy == -1)
        {
            // Vecino abajo (Sur)
            if (currentCell.Walls[1] != null) Destroy(currentCell.Walls[1]);
            if (neighborCell.Walls[0] != null) Destroy(neighborCell.Walls[0]);
        }
    }

    // ===================== A* PATHFINDING =====================

    private float Heuristic(Cell a, Cell b)
    {
        int dx = Mathf.Abs(a.X - b.X);
        int dy = Mathf.Abs(a.Y - b.Y);
        return dx + dy; // Distancia Manhattan
    }

    private List<Cell> FindPathAStar(MazeInstance instance, Cell start, Cell goal)
    {
        List<AStarNode> openList = new();
        HashSet<Cell> closedSet = new();

        AStarNode startNode = new AStarNode(start)
        {
            G = 0f,
            H = Heuristic(start, goal)
        };
        openList.Add(startNode);

        while (openList.Count > 0)
        {
            // Elegimos el nodo con menor F
            AStarNode current = openList[0];
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].F < current.F)
                {
                    current = openList[i];
                }
            }

            if (current.Cell == goal)
            {
                List<Cell> path = ReconstructPath(current);
                return path;
            }

            openList.Remove(current);
            closedSet.Add(current.Cell);

            foreach (Vector2Int dir in Directions)
            {
                int nx = current.Cell.X + dir.x;
                int ny = current.Cell.Y + dir.y;

                if (!IsInsideBounds(nx, ny))
                    continue;

                Cell neighborCell = instance.Grid[nx, ny];

                // Solo avanzamos si hay conexión (no hay muro entre ellos)
                if (!AreCellsConnected(current.Cell, neighborCell))
                    continue;

                if (closedSet.Contains(neighborCell))
                    continue;

                float tentativeG = current.G + 1f; // todas las aristas valen 1

                AStarNode neighborNode = null;
                for (int i = 0; i < openList.Count; i++)
                {
                    if (openList[i].Cell == neighborCell)
                    {
                        neighborNode = openList[i];
                        break;
                    }
                }

                if (neighborNode == null)
                {
                    neighborNode = new AStarNode(neighborCell);
                    neighborNode.G = tentativeG;
                    neighborNode.H = Heuristic(neighborCell, goal);
                    neighborNode.Parent = current;
                    openList.Add(neighborNode);
                }
                else if (tentativeG < neighborNode.G)
                {
                    neighborNode.G = tentativeG;
                    neighborNode.H = Heuristic(neighborCell, goal);
                    neighborNode.Parent = current;
                }
            }
        }

        Debug.LogWarning("A*: No se encontró camino (no debería pasar en un laberinto perfecto).");
        return null;
    }

    private bool AreCellsConnected(Cell a, Cell b)
    {
        int dx = b.X - a.X;
        int dy = b.Y - a.Y;

        // Destroy() en Unity hace que el objeto == null aunque la referencia exista
        if (dx == 1)
        {
            // b está a la derecha (Este)
            return (a.Walls[2] == null && b.Walls[3] == null);
        }
        if (dx == -1)
        {
            // b a la izquierda (Oeste)
            return (a.Walls[3] == null && b.Walls[2] == null);
        }
        if (dy == 1)
        {
            // b arriba (Norte)
            return (a.Walls[0] == null && b.Walls[1] == null);
        }
        if (dy == -1)
        {
            // b abajo (Sur)
            return (a.Walls[1] == null && b.Walls[0] == null);
        }

        return false;
    }

    private List<Cell> ReconstructPath(AStarNode endNode)
    {
        List<Cell> path = new();
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
        if (path == null || path.Count == 0 || pathMaterial == null)
            return;

        foreach (Cell cell in path)
        {
            // No tocamos inicio ni fin (ya tienen sus materiales)
            if (cell == start || cell == end)
                continue;

            if (cell.CellObject == null)
                continue;

            Transform ground = cell.CellObject.transform.Find("Ground");
            if (ground == null)
                continue;

            Renderer renderer = ground.GetComponent<Renderer>();
            if (renderer == null)
                continue;

            renderer.material = pathMaterial;
        }
    }

    // ===================== CAMPO DE DISTANCIAS PARA RL =====================

    private void ComputeDistanceField(MazeInstance instance)
    {
        if (instance.EndCell == null)
        {
            Debug.LogWarning("ComputeDistanceField: EndCell es null.");
            return;
        }

        int[,] field = new int[width, height];

        // Inicializamos con "infinito"
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                field[x, y] = int.MaxValue;
            }
        }

        Queue<Cell> queue = new();
        Cell goal = instance.EndCell;

        field[goal.X, goal.Y] = 0;
        queue.Enqueue(goal);

        while (queue.Count > 0)
        {
            Cell current = queue.Dequeue();
            int currentDist = field[current.X, current.Y];

            foreach (Vector2Int dir in Directions)
            {
                int nx = current.X + dir.x;
                int ny = current.Y + dir.y;

                if (!IsInsideBounds(nx, ny))
                    continue;

                Cell neighbor = instance.Grid[nx, ny];

                // Solo consideramos vecinos conectados (sin muro)
                if (!AreCellsConnected(current, neighbor))
                    continue;

                int newDist = currentDist + 1;
                if (newDist < field[nx, ny])
                {
                    field[nx, ny] = newDist;
                    queue.Enqueue(neighbor);
                }
            }
        }

        // Guardamos en el diccionario para este laberinto
        mazeDistanceFields[instance] = field;

        // Si este laberinto es el principal, actualizamos el campo global (legacy)
        if (instance == mainInstance)
        {
            distanceField = field;
        }
    }

    // ===================== API PÚBLICA PARA EL AGENTE =====================

    public bool HasAnyDistanceField => mazeDistanceFields.Count > 0;

    // Posición mundial de la celda objetivo del laberinto principal (legacy)
    public Vector3 GetGoalWorldPos()
    {
        if (mainInstance == null || mainInstance.EndCell == null)
            return Vector3.zero;

        return mainInstance.EndCell.CellObject.transform.position;
    }

    /// <summary>
    /// Goal del laberinto específico de un agente (por índice).
    /// </summary>
    public Vector3 GetGoalWorldPosForMaze(int mazeIndex)
    {
        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
            return Vector3.zero;

        MazeInstance instance = mazes[mazeIndex];
        if (instance.EndCell == null || instance.EndCell.CellObject == null)
            return Vector3.zero;

        return instance.EndCell.CellObject.transform.position;
    }

    /// <summary>
    /// Distancia desde worldPos hasta el goal, dentro del laberinto con ese índice.
    /// </summary>
    public int GetDistanceFromWorldPos(Vector3 worldPos, int mazeIndex)
    {
        if (!HasAnyDistanceField)
            return int.MaxValue;

        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
            return int.MaxValue;

        MazeInstance instance = mazes[mazeIndex];

        if (!WorldToCell(instance, worldPos, out int gx, out int gy))
            return int.MaxValue;

        if (!mazeDistanceFields.TryGetValue(instance, out int[,] field))
            return int.MaxValue;

        return field[gx, gy];
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

        return IsInsideBounds(gx, gy);
    }

    /// <summary>
    /// Devuelve la lista de MeshRenderer de Ground de un laberinto dado (por índice),
    /// excluyendo inicio, fin y el camino A*.
    /// </summary>
    public List<MeshRenderer> GetMazeFloorRenderers(int mazeIndex)
    {
        var result = new List<MeshRenderer>();

        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
            return result;

        MazeInstance instance = mazes[mazeIndex];
        mazePaths.TryGetValue(instance, out List<Cell> path);

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                Cell cell = instance.Grid[x, y];
                if (cell == null || cell.CellObject == null)
                    continue;

                // No tocar inicio ni fin
                if (cell == instance.StartCell || cell == instance.EndCell)
                    continue;

                // No tocar celdas que forman el camino A*
                if (path != null && path.Contains(cell))
                    continue;

                Transform ground = cell.CellObject.transform.Find("Ground");
                if (ground == null)
                    continue;

                var renderer = ground.GetComponent<MeshRenderer>();
                if (renderer != null)
                {
                    result.Add(renderer);
                }
            }
        }

        return result;
    }

    /// <summary>
    /// Devuelve el camino A* del laberinto indicado, como lista de posiciones de mundo (centro de cada celda).
    /// </summary>
    public List<Vector3> GetPathForMaze(int mazeIndex)
    {
        if (mazeIndex < 0 || mazeIndex >= mazes.Count)
            return null;

        MazeInstance instance = mazes[mazeIndex];

        if (!mazePaths.TryGetValue(instance, out List<Cell> pathCells) || pathCells == null || pathCells.Count == 0)
            return null;

        var result = new List<Vector3>(pathCells.Count);
        for (int i = 0; i < pathCells.Count; i++)
        {
            Cell cell = pathCells[i];
            if (cell?.CellObject == null) continue;
            result.Add(cell.CellObject.transform.position);
        }

        return result;
    }

    /// <summary>
    /// Devuelve la posición de spawn (inicio) para el laberinto indicado.
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
}
