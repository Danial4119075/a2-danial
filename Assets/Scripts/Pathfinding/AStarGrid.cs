// Adapted from: https://github.com/SebLague/Pathfinding-2D

using UnityEngine;
using System.Collections.Generic;

public class AStarGrid : MonoBehaviour
{
    public bool displayGridGizmos;

    // ---- Obstacle detection ----
    // Objects on this layer are treated as solid walls — the node is marked unwalkable.
    // UNITY SETUP: the "Obstacle" layer already exists. Rocks and Trees are on it.
    public LayerMask unwalkableMask;

    // ---- Terrain detection (Task 2 — Varying Terrain) ----
    // Objects on this layer slow movement but don't block it entirely.
    // UNITY SETUP:
    //   1. Edit > Project Settings > Tags and Layers
    //   2. Add a new User Layer called exactly "Mud"
    //   3. Select each MudPatch GameObject in the Hierarchy
    //   4. Change its Layer (top of Inspector) to "Mud"
    //   5. Back here, set this LayerMask field to "Mud" in the Inspector
    public LayerMask terrainMask;

    // How much more expensive mud is to cross compared to normal ground.
    // A value of 3 means A* will prefer a 3x-longer grass path over cutting through mud.
    // Visible effect: frog detours around mud when a roughly equal grass route exists.
    // Try values between 2 and 5. Default 3 is a good starting point for the demo.
    public float mudPenalty = 3f;

    public Vector2 gridWorldSize;
    public float gridSize;
    public float overlapCircleRadius;

    public bool includeDiagonalNeighbours;

    Node[,] grid;
    float nodeDiameter;
    int gridSizeX, gridSizeY;

    void Awake()
    {
        nodeDiameter = gridSize * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);

        CreateGrid();
    }

    public int MaxSize
    {
        get
        {
            return gridSizeX * gridSizeY;
        }
    }

    public void CreateGrid()
    {
        grid = new Node[gridSizeX, gridSizeY];
        Vector2 worldBottomLeft = (Vector2)transform.position - Vector2.right * gridWorldSize.x / 2 - Vector2.up * gridWorldSize.y / 2;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector2 worldPoint = worldBottomLeft + Vector2.right * (x * nodeDiameter + gridSize) + Vector2.up * (y * nodeDiameter + gridSize);

                // A node is unwalkable if any obstacle collider overlaps its centre point.
                // This mirrors how the assignment spec says to detect terrain type —
                // using the same OverlapCircle approach, just with a different LayerMask.
                bool walkable = (Physics2D.OverlapCircle(worldPoint, overlapCircleRadius, unwalkableMask) == null);

                // Task 2 — Varying Terrain:
                // Check whether this node sits on slow terrain (e.g. mud).
                // If so, assign a movement penalty so A* prefers grass routes.
                // A node can be on mud AND be unwalkable (e.g. a rock sitting on mud) —
                // in that case walkable = false takes priority and the penalty is irrelevant.
                float penalty = 1.0f; // default: normal terrain, no extra cost
                if (walkable && Physics2D.OverlapCircle(worldPoint, overlapCircleRadius, terrainMask) != null)
                {
                    penalty = mudPenalty; // mud: movement is penalised
                }

                grid[x, y] = new Node(walkable, worldPoint, x, y, penalty);
            }
        }
    }

    public List<Node> GetNeighbours(Node node)
    {
        List<Node> neighbours = new List<Node>();

        // Loop over the 3x3 block of cells surrounding the node.
        // dx and dy each range from -1 (one cell left/down) to +1 (one cell right/up).
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                // Skip the centre cell — that's the node itself, not a neighbour.
                if (dx == 0 && dy == 0)
                    continue;

                // If diagonal movement is disabled, skip cells where BOTH dx and dy
                // are non-zero (those are the four corner/diagonal cells).
                // When disabled, only the four cardinal directions (up/down/left/right)
                // are returned, each of which has exactly one of dx or dy equal to zero.
                if (!includeDiagonalNeighbours && dx != 0 && dy != 0)
                    continue;

                int nx = node.gridX + dx;
                int ny = node.gridY + dy;

                // BUG FIX: the original code used "> 0" for the lower-bound check,
                // which excluded index 0 (the bottom and left edges of the grid).
                // The correct check is ">= 0" so every grid cell is reachable.
                if (!InBounds(nx, ny))
                    continue;

                // CORNER-CUTTING FIX: for diagonal moves, also require that both
                // adjacent cardinal cells are walkable.
                //
                // Example: moving diagonally from (x,y) to (x+1,y+1).
                //   We check (x+1,y) and (x,y+1) are both walkable.
                //   If either cardinal neighbour has an obstacle, the frog's physical
                //   collider cannot squeeze through the corner gap, even though the
                //   diagonal cell itself is technically walkable. Allowing such moves
                //   causes the frog to walk into a corner and get stuck.
                if (dx != 0 && dy != 0)
                {
                    bool cardinalXWalkable = InBounds(node.gridX + dx, node.gridY)
                                            && grid[node.gridX + dx, node.gridY].walkable;
                    bool cardinalYWalkable = InBounds(node.gridX, node.gridY + dy)
                                            && grid[node.gridX, node.gridY + dy].walkable;
                    if (!cardinalXWalkable || !cardinalYWalkable)
                        continue;
                }

                neighbours.Add(grid[nx, ny]);
            }
        }

        return neighbours;
    }

    public Node NodeFromWorldPoint(Vector2 worldPosition)
    {
        float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
        float percentY = (worldPosition.y + gridWorldSize.y / 2) / gridWorldSize.y;
        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }

    public Node ClosestWalkableNode(Node node)
    {
        int maxRadius = Mathf.Max(gridSizeX, gridSizeY) / 2;
        for (int i = 1; i < maxRadius; i++)
        {
            Node n = FindWalkableInRadius(node.gridX, node.gridY, i);
            if (n != null)
            {
                return n;
            }
        }
        return null;
    }

    Node FindWalkableInRadius(int centreX, int centreY, int radius)
    {
        for (int i = -radius; i <= radius; i++)
        {
            int verticalSearchX = i + centreX;
            int horizontalSearchY = i + centreY;

            // Top
            if (InBounds(verticalSearchX, centreY + radius))
            {
                if (grid[verticalSearchX, centreY + radius].walkable)
                {
                    return grid[verticalSearchX, centreY + radius];
                }
            }

            // Bottom
            if (InBounds(verticalSearchX, centreY - radius))
            {
                if (grid[verticalSearchX, centreY - radius].walkable)
                {
                    return grid[verticalSearchX, centreY - radius];
                }
            }

            // Right
            if (InBounds(centreY + radius, horizontalSearchY))
            {
                if (grid[centreX + radius, horizontalSearchY].walkable)
                {
                    return grid[centreX + radius, horizontalSearchY];
                }
            }

            // Left
            if (InBounds(centreY - radius, horizontalSearchY))
            {
                if (grid[centreX - radius, horizontalSearchY].walkable)
                {
                    return grid[centreX - radius, horizontalSearchY];
                }
            }

        }

        return null;
    }

    bool InBounds(int x, int y)
    {
        return x >= 0 && x < gridSizeX && y >= 0 && y < gridSizeY;
    }

    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector2(gridWorldSize.x, gridWorldSize.y));

        if (grid != null && displayGridGizmos)
        {
            foreach (Node n in grid)
            {
                // Colour scheme (matches assignment spec Figure 2):
                //   Red   = unwalkable (obstacle — rock, tree)
                //   Blue  = slow terrain (mud — walkable but penalised)
                //   Grey  = normal grass (walkable, no penalty)
                if (!n.walkable)
                {
                    Gizmos.color = Color.red;
                }
                else if (n.movementPenalty > 1.0f)
                {
                    Gizmos.color = Color.blue;
                }
                else
                {
                    Gizmos.color = Color.grey;
                }

                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
            }
        }
    }
}
