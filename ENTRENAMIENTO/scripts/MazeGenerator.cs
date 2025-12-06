using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MazeGenerator : MonoBehaviour
{
    [Header("Prefab y dimensiones")]
    public GameObject cellPrefab;      // Prefab que contiene el suelo y las 4 paredes.
    public int width = 5;              // Número de celdas en el ancho del laberinto
    public int height = 5;             // Número de celdas en la altura del laberinto
    public float cellSizeX = 5.0f;     // Tamaño de la celda en el eje X (ajustable según tu prefab)
    public float cellSizeZ = 5.0f;     // Tamaño de la celda en el eje Z (ajustable según tu prefab)

    [Header("Multiples laberintos")]
    public int mazeRows = 1;           // Número de laberintos en filas
    public int mazeColumns = 1;        // Número de laberintos en columnas
    public float mazePadding = 2.0f;   // Espacio entre laberintos

    [Header("Materiales")]
    public Material startMaterial;     // Material para la celda de inicio
    public Material endMaterial;       // Material para la celda de fin

    [Header("Personajes")]
    public GameObject playerPrefab;    // Prefab del personaje a instanciar en el inicio
    public float playerSpawnYOffset = 1.0f; // Altura de spawn respecto al suelo

    private readonly List<MazeInstance> mazes = new();

    private static readonly Vector2Int[] Directions =
    {
        new(1, 0),   // Este
        new(-1, 0),  // Oeste
        new(0, 1),   // Norte
        new(0, -1)   // Sur
    };

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

        public MazeInstance(int width, int height, Vector3 origin)
        {
            Grid = new Cell[width, height];
            Origin = origin;
        }
    }

    // Genera todos los laberintos solicitados en una grilla
    private void GenerateAllMazes()
    {
        mazes.Clear();

        float mazeTotalWidth = width * cellSizeX + mazePadding;
        float mazeTotalHeight = height * cellSizeZ + mazePadding;

        for (int row = 0; row < mazeRows; row++)
        {
            for (int col = 0; col < mazeColumns; col++)
            {
                Vector3 origin = new(col * mazeTotalWidth, 0f, row * mazeTotalHeight);
                MazeInstance instance = new(width, height, origin);
                mazes.Add(instance);
                BuildMaze(instance);
                StartCoroutine(GenerateMazeUsingPrim(instance));
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
                GameObject cellObject = Instantiate(cellPrefab, cellPosition, Quaternion.identity, transform);
                GameObject[] walls = GetWalls(cellObject);

                instance.Grid[x, y] = new Cell(walls, cellObject, x, y);

                if (x == 0 && y == 0)
                {
                    SetMaterial(cellObject, startMaterial);
                }
                else if (x == width - 1 && y == height - 1)
                {
                    SetMaterial(cellObject, endMaterial);
                }
            }
        }

        SpawnPlayer(instance);
    }

    // Instancia un jugador en la celda inicial del laberinto
    private void SpawnPlayer(MazeInstance instance)
    {
        if (playerPrefab == null)
        {
            return;
        }

        Vector3 spawnPosition = instance.Origin + new Vector3(0f, playerSpawnYOffset, 0f);
        Instantiate(playerPrefab, spawnPosition, Quaternion.identity, transform);
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
            if (renderer != null)
            {
                renderer.material = material;
            }
            else
            {
                Debug.LogWarning("El objeto 'Ground' no tiene un componente Renderer.");
            }
        }
        else
        {
            Debug.LogWarning("El objeto 'Ground' no se encuentra dentro de la celda.");
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
    }

    // Función para añadir las celdas vecinas no visitadas a la frontera
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

    // Conecta la celda actual con un vecino ya visitado, derribando la pared compartida
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
            Destroy(currentCell.Walls[2]); // Este
            Destroy(neighborCell.Walls[3]); // Oeste
        }
        else if (dx == -1)
        {
            Destroy(currentCell.Walls[3]); // Oeste
            Destroy(neighborCell.Walls[2]); // Este
        }
        else if (dy == 1)
        {
            Destroy(currentCell.Walls[0]); // Norte
            Destroy(neighborCell.Walls[1]); // Sur
        }
        else if (dy == -1)
        {
            Destroy(currentCell.Walls[1]); // Sur
            Destroy(neighborCell.Walls[0]); // Norte
        }
    }

    private bool IsInsideBounds(int x, int y)
    {
        return x >= 0 && y >= 0 && x < width && y < height;
    }
}