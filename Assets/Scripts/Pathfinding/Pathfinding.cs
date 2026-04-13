// Adapted from: https://github.com/SebLague/Pathfinding-2D

using UnityEngine;
using System.Collections.Generic;
using System;

public class Pathfinding : MonoBehaviour
{
    public static AStarGrid grid;
    static Pathfinding instance;



    // This is used instead of Start() so that the
    // A* grid is only greated once when the game is launched
    void Awake()
    {
        grid = GetComponent<AStarGrid>();
        instance = this;
    }

    // Public callable method
    public static Node[] RequestPath(Vector2 from, Vector2 to)
    {
        return instance.FindPath(from, to);
    }


    // Internal private implementation
    Node[] FindPath(Vector2 from, Vector2 to)
    {
        // A* Waypoints to return
        Node[] waypoints = new Node[0];

        // Set to true if a path is found
        bool pathSuccess = false;

        // Starting node point - selected from the A* Grid
        Node startNode = grid.NodeFromWorldPoint(from);

        // Goal node point - selected from the A* Grid
        Node targetNode = grid.NodeFromWorldPoint(to);

        // Ensure the starting node's parent is not null
        // Also let's us detect the start node if needed
        startNode.parent = startNode;

        // Niceity check to ensure the start and target nodes are walkable
        // by the frog (such as if you clock on a object)
        // If not, we find the closest walkable point in the grid
        if (!startNode.walkable)
        {
            startNode = grid.ClosestWalkableNode(startNode);
        }
        if (!targetNode.walkable)
        {
            targetNode = grid.ClosestWalkableNode(targetNode);
        }

        if (startNode.walkable && targetNode.walkable)
        {
            // A* Starts here!!!
            // TODO: Your job is to fill in the missing code below the marked comments

            // Track the open set of nodes to explore, as a heap sorted by the A* Cost
            Heap<Node> openSet = new Heap<Node>(grid.MaxSize);

            // Track closed set of all visited nodes
            HashSet<Node> closedSet = new HashSet<Node>();

            // TODO: Commence A* by adding the start node to the open set
            

            // Stop if we have a path or run out of nodes to explore (means no path can be found!)
            while (!pathSuccess && openSet.Count > 0)
            {
                // TODO: Get the node with the lowest F cost from the open set
                //     and add it to the closed set
                

                // TODO: If we have reached the target node, we have found a path! (repalce false)
                if (false)
                {
                    pathSuccess = true;
                }
                else
                {
                    // TODO: Otherwise, explore the neighbours of the current node
                    //       You'll need to get all of the neighbours of the current node
                    //       and then loop through them to find the best path
                    Node[] neighbours = new Node[0];
                    foreach (Node node in neighbours)
                    {
                        // TODO:If we can reach the neighbour and it is not in the closed set (repalce false)
                        if (false)
                        {
                            // TODO: Calculate the G Cost of the neighbour node


                            // TOSO: If the neighbour is not in the open set OR
                            //    the neighour was previously checked and the new G Cost is less than the previous G Cost 
                            //    (repalce false)
                            if (false)
                            {
                                // TODO: Set neightbour G Cost
                                

                                // TODO: Compute and set the H Cost for the neighbour
                                

                                // TODO: Set the parent of the neighbour to the current node
                                

                                // TODO: Add neighbour to the open set, but need to check if the neighbour is already in the open set
                                // If not in the open set, then add to the heap
                                // If in the open set, then UDPATE the neighbour in the heap
                                if (false)
                                {
                                    // TODO: (see above comment)
                                }
                                else
                                {
                                    // TODO: (see above comment)
                                }
                            }
                        }
                    }
                }
            }
        }

        // If we have a path, then actually get the path from the start to goal
        if (pathSuccess)
        {
            waypoints = RetracePath(startNode, targetNode);
        }

        return waypoints;
    }

    // Creates the actual A* Path from the start to the goal
    // TODO: Your job is to fill in the missing code below the marked comments
    Node[] RetracePath(Node startNode, Node endNode)
    {
        // Store the computed path
        List<Node> path = new List<Node>();

        // TODO: Commence retracing the path from the end node
        

        // TODO: Loop while the current node isn't the start node (replace false)
        while (false)
        {
            // TODO: Add the current node to the path
            

            // TODO: Set the current node to the parent of the current node
            
        }

        // Convert this list to an array and reverse it
        Node[] waypoints = path.ToArray();
        Array.Reverse(waypoints);
        return waypoints;
    }

    private float GCost(Node nodeA, Node nodeB)
    {
        return 1.0f;
    }


    private float Heuristic(Node nodeA, Node nodeB)
    {
        // TODO: Implement A* heuristics, such as Manhattan distance or Eculedian distance
        return 0;
    }
}
