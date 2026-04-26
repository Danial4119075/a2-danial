// Adapted from: https://github.com/SebLague/Pathfinding-2D

using UnityEngine;
using System.Collections.Generic;
using System;

// ---------- Task 3: Varying Heuristics ----------
// Selectable from the Unity Inspector on the Pathfinding GameObject.
// Each option changes which h(n) estimate A* uses to prioritise nodes.
// Switch between them at runtime to see how each affects the path shape
// and the number of nodes explored (visible via the green debug lines).
public enum HeuristicType
{
    Manhattan,          // h = |dx| + |dy|      — NOT admissible with diagonal movement
    Euclidean,          // h = sqrt(dx²+dy²)    — admissible, our default
    DiagonalProjection  // h = (dx+dy) / √2     — our custom admissible heuristic
}

public class Pathfinding : MonoBehaviour
{
    // Reference to the grid component on the same GameObject.
    // Static so any script can call RequestPath() without needing a direct reference.
    public static AStarGrid grid;
    static Pathfinding instance;

    // ---------- Task 3: Heuristic selector ----------
    // Change this in the Inspector while the game is running to compare heuristics.
    // Default is Euclidean — the tightest admissible option for diagonal grids.
    public HeuristicType heuristic = HeuristicType.Euclidean;

    // Awake() is used instead of Start() so the grid is initialised before
    // any other script tries to call RequestPath() in their Start().
    void Awake()
    {
        grid = GetComponent<AStarGrid>();
        instance = this;
    }

    // ---------- Public API ----------

    // Any script (e.g. Frog.cs) calls this to get an A* path.
    // Returns an array of Nodes from start to destination, or an empty array if no path exists.
    public static Node[] RequestPath(Vector2 from, Vector2 to)
    {
        return instance.FindPath(from, to);
    }

    // ---------- Core A* Implementation ----------

    // FindPath runs the A* search algorithm on the grid.
    // 'from' and 'to' are world-space positions; we convert them to grid nodes first.
    Node[] FindPath(Vector2 from, Vector2 to)
    {
        // The path we will return. Stays empty if no path is found.
        Node[] waypoints = new Node[0];

        // Becomes true once we have traced a complete route to the target.
        bool pathSuccess = false;

        // Convert world positions to the nearest grid node.
        Node startNode  = grid.NodeFromWorldPoint(from);
        Node targetNode = grid.NodeFromWorldPoint(to);

        // Setting the start node's parent to itself lets RetracePath know
        // when to stop walking up the parent chain.
        startNode.parent = startNode;

        // Safety: if the frog is sitting on an unwalkable cell (e.g. slightly
        // overlapping a rock) or the player clicked on a tree, snap to the
        // nearest walkable node so A* can still run.
        if (!startNode.walkable)
            startNode = grid.ClosestWalkableNode(startNode);
        if (!targetNode.walkable)
            targetNode = grid.ClosestWalkableNode(targetNode);

        // Only run A* if both endpoints are actually reachable.
        if (startNode.walkable && targetNode.walkable)
        {
            // ----- A* OPEN / CLOSED SETS -----

            // The open set is a min-heap sorted by f(n) = g(n) + h(n).
            // RemoveFirst() always gives us the node with the lowest estimated
            // total cost — the most promising candidate to expand next.
            Heap<Node> openSet = new Heap<Node>(grid.MaxSize);

            // The closed set holds nodes we have already expanded.
            // HashSet gives O(1) membership checks, which matters in large grids.
            HashSet<Node> closedSet = new HashSet<Node>();

            // Step 1 — Seed the search with the start node (g=0, h=heuristic).
            openSet.Add(startNode);

            // Keep going while there are candidates to explore and we haven't
            // found the goal yet. If the open set empties without success, no
            // path exists (the target is completely surrounded by obstacles).
            while (!pathSuccess && openSet.Count > 0)
            {
                // Step 2 — Pick the node with the lowest f(n).
                // RemoveFirst() pops the top of the min-heap in O(log n).
                Node currentNode = openSet.RemoveFirst();

                // This node is now fully explored — move it to the closed set.
                closedSet.Add(currentNode);

                // Step 3 — Check if we have reached the destination.
                if (currentNode == targetNode)
                {
                    pathSuccess = true; // exit the while loop
                }
                else
                {
                    // Step 4 — Expand the current node by examining its neighbours.
                    // GetNeighbours returns 4 (cardinal) or 8 (cardinal + diagonal)
                    // neighbours depending on the AStarGrid.includeDiagonalNeighbours flag.
                    List<Node> neighbours = grid.GetNeighbours(currentNode);

                    foreach (Node neighbour in neighbours)
                    {
                        // Skip this neighbour if it has an obstacle on it,
                        // or if we have already fully processed it.
                        if (!neighbour.walkable || closedSet.Contains(neighbour))
                            continue;

                        // Step 5 — Calculate the tentative g cost to reach this
                        // neighbour via the current node.
                        //   g(neighbour) = g(current) + move cost(current → neighbour)
                        // GCost() returns 1.0 for cardinal moves and √2 ≈ 1.414 for
                        // diagonals, scaled by the destination's terrain penalty.
                        float tentativeGCost = currentNode.gCost + GCost(currentNode, neighbour);

                        // Step 6 — Decide whether to update this neighbour.
                        // We update it if:
                        //   (a) it has never been added to the open set yet, OR
                        //   (b) we found a strictly cheaper route to it than before.
                        bool inOpenSet = openSet.Contains(neighbour);
                        if (tentativeGCost < neighbour.gCost || !inOpenSet)
                        {
                            // Record the cheapest known cost to reach this neighbour.
                            neighbour.gCost = tentativeGCost;

                            // h(n): estimate of remaining cost to the goal.
                            // Which formula is used depends on the heuristic field (Task 3).
                            neighbour.hCost = Heuristic(neighbour, targetNode);

                            // Record which node we came from so we can reconstruct the path later.
                            neighbour.parent = currentNode;

                            if (!inOpenSet)
                            {
                                // First time discovering this node — add it to the heap.
                                openSet.Add(neighbour);
                            }
                            else
                            {
                                // Already in the heap but we found a cheaper route.
                                // UpdateItem re-sorts it upward so the heap stays valid.
                                openSet.UpdateItem(neighbour);
                            }
                        }
                    }
                }
            }
        }

        // If A* reached the target, trace parent pointers back from
        // target → start and reverse the list to get start → target.
        if (pathSuccess)
        {
            waypoints = RetracePath(startNode, targetNode);
        }

        return waypoints;
    }

    // ---------- Path Reconstruction ----------

    // Walks back from endNode to startNode by following each node's parent pointer,
    // collects the nodes into a list, then reverses it so it runs start → target.
    Node[] RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();

        // Start at the goal and walk backwards up the parent chain.
        Node currentNode = endNode;

        // Stop when we reach the start node (whose parent points to itself).
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }

        // Convert to array and reverse so waypoints go start → target.
        Node[] waypoints = path.ToArray();
        Array.Reverse(waypoints);
        return waypoints;
    }

    // ---------- Cost Functions ----------

    // Returns the movement cost of stepping from nodeA into nodeB.
    //
    // Two factors combine:
    //
    // 1) Direction cost — diagonal vs cardinal:
    //    Diagonal steps cover √2 ≈ 1.414 units of actual distance; cardinal steps
    //    cover 1 unit. Using 1.0 for every move would make diagonals "free" and
    //    produce geometrically incorrect paths.
    //
    // 2) Terrain penalty — from nodeB.movementPenalty (Task 2 — Varying Terrain):
    //    Normal grass has penalty 1.0 (no change). Mud has penalty > 1.0, making
    //    each step through mud proportionally more expensive. A* then naturally
    //    routes around mud when an equivalent or shorter grass path exists.
    //    The penalty applies to the DESTINATION node (nodeB) because we pay the
    //    terrain cost of the cell we are entering.
    private float GCost(Node nodeA, Node nodeB)
    {
        int dx = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dy = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        // Base movement cost: √2 for diagonal, 1.0 for cardinal.
        float moveCost = (dx == 1 && dy == 1) ? 1.414f : 1.0f;

        // Multiply by the destination node's terrain penalty.
        // On normal grass: penalty = 1.0, so moveCost is unchanged.
        // On mud:          penalty = mudPenalty (e.g. 3.0), so the step costs 3x more.
        return moveCost * nodeB.movementPenalty;
    }

    // ---------- Task 3: Heuristic Functions ----------
    //
    // h(n) estimates the remaining cost from node nodeA to the goal nodeB.
    // A* expands nodes in order of f(n) = g(n) + h(n), so h(n) directly controls
    // which parts of the grid get explored first.
    //
    // ADMISSIBILITY REQUIREMENT:
    //   h(n) must NEVER overestimate the true remaining cost, otherwise A*
    //   may skip a cheaper path and return a suboptimal result.
    //
    // INTERACTION WITH TERRAIN:
    //   All three heuristics only use grid-cell distances (dx, dy). They do NOT
    //   know about terrain penalties ahead. This means on maps with mud, the true
    //   cost is higher than the heuristic estimates — the heuristics underestimate
    //   MORE than usual. A* remains correct (admissibility is preserved because
    //   minimum terrain penalty = 1.0 ≥ 1.0), but explores extra nodes around mud
    //   before finding the detour. A terrain-aware heuristic could be tighter, but
    //   requires scanning the grid ahead — a more complex design not required here.
    //
    private float Heuristic(Node nodeA, Node nodeB)
    {
        int dx = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dy = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        switch (heuristic)
        {
            // ------------------------------------------------------------------
            // MANHATTAN DISTANCE   h = |dx| + |dy|
            // ------------------------------------------------------------------
            // Counts total axis-aligned steps as if only cardinal movement exists.
            //
            // With CARDINAL-ONLY movement: admissible and exact (no diagonals
            //   possible, so the minimum steps really is dx + dy at cost 1 each).
            //
            // With DIAGONAL movement: INADMISSIBLE — overestimates badly.
            //   Example: goal is 3 right, 3 up. Manhattan = 6.
            //   But 3 diagonal moves get us there for cost 3 × √2 ≈ 4.24.
            //   Manhattan overestimates by ~42%, so A* may return suboptimal paths.
            //
            // DEMO VALUE: switch to Manhattan with diagonals enabled and watch the
            //   frog take visibly wrong (non-shortest) routes. This illustrates why
            //   admissibility matters for correctness.
            //
            // WITH TERRAIN: overestimates even more on diagonal grids, but on
            //   cardinal-only grids still admissible (terrain penalty ≥ 1.0, so
            //   true cost ≥ dx + dy = Manhattan).
            case HeuristicType.Manhattan:
                return dx + dy;

            // ------------------------------------------------------------------
            // EUCLIDEAN DISTANCE   h = sqrt(dx² + dy²)
            // ------------------------------------------------------------------
            // Straight-line distance between the two grid cells.
            //
            // With DIAGONAL movement: admissible and relatively tight.
            //   The true cost ≥ Euclidean (no path can be shorter than a straight
            //   line), so h never overestimates. For pure diagonal displacements
            //   (dx == dy), Euclidean equals the true cost exactly — it is tight.
            //
            // With CARDINAL-ONLY movement: admissible but LOOSE (underestimates
            //   heavily). For dx=3, dy=3: Euclidean ≈ 4.24, true cost = 6.
            //   A* explores many extra nodes because h is far from the true cost.
            //
            // WITH TERRAIN: still admissible because minimum terrain penalty = 1.0.
            //   Mud cells have penalty > 1.0, so true costs rise while h stays the
            //   same — A* correctly explores more nodes to find the detour.
            //
            // This is our DEFAULT and the best general-purpose choice.
            case HeuristicType.Euclidean:
                return Mathf.Sqrt(dx * dx + dy * dy);

            // ------------------------------------------------------------------
            // DIAGONAL PROJECTION   h = (dx + dy) / √2       [OUR CUSTOM HEURISTIC]
            // ------------------------------------------------------------------
            // INVENTION:
            //   We project the displacement vector (dx, dy) onto the 45° diagonal
            //   direction — the single most efficient movement direction on this grid.
            //   The projection length is (dx + dy) / √2 (from the vector dot product).
            //
            // WHY ADMISSIBLE:
            //   By the Cauchy-Schwarz inequality, projecting any vector onto a unit
            //   vector always gives a result ≤ the vector's magnitude. Therefore:
            //
            //     h = (dx + dy) / √2  ≤  √(dx² + dy²)  =  Euclidean  ≤  true cost
            //
            //   The middle inequality holds because (dx+dy)²/2 ≤ dx²+dy² is
            //   equivalent to 0 ≤ (dx−dy)², which is always true.
            //   Since true cost ≥ Euclidean ≥ h, the heuristic never overestimates
            //   → it is admissible for both cardinal and diagonal grids.
            //
            // SPECIAL PROPERTY — exact for pure diagonal displacements:
            //   When dx == dy (goal is at 45° from current node):
            //     h = 2dx / √2 = dx√2 = Euclidean = true optimal cost.
            //   The heuristic is TIGHT in this case — A* explores the fewest nodes.
            //   When dx ≠ dy, h = Euclidean / √2 (roughly 70% of Euclidean),
            //   so A* explores slightly more nodes than Euclidean but still finds
            //   the optimal path.
            //
            // INTERACTION WITH DIAGONAL MOVEMENT:
            //   Because h is tightest when the goal is diagonally ahead, A* focuses
            //   search along the diagonal axis first. Paths that require a lot of
            //   cardinal-only stretches (dx >> dy or dy >> dx) cause h to
            //   underestimate more, widening the search front — visible as more
            //   green debug lines off the direct route.
            //
            // INTERACTION WITH TERRAIN:
            //   Same argument as Euclidean: minimum terrain penalty = 1.0, so true
            //   cost ≥ Euclidean ≥ h. Admissibility is preserved. The heuristic is
            //   a LOOSER lower bound than Euclidean, so A* may explore slightly
            //   more nodes near mud patches before committing to a detour.
            //
            // DEMO VALUE:
            //   Click a goal diagonally ahead of the frog → almost no extra nodes
            //   explored (tight bound). Click a goal directly left/right →
            //   noticeably wider search (loose bound). Compare with Euclidean to
            //   see the difference in explored area.
            case HeuristicType.DiagonalProjection:
                return (dx + dy) / 1.414f;

            default:
                return Mathf.Sqrt(dx * dx + dy * dy);
        }
    }
}
