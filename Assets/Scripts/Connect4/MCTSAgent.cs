using UnityEngine;
using System.Collections.Generic;

public class MCTSAgent : Agent
{
    public int totalSims = 2500;
    public float c = Mathf.Sqrt(2.0f);

    public override int GetMove(Connect4State state)
    {
        // TODO: Override this method with an MCTS implementation.
        // Currently, it just returns a random move.
        // You can add other methods to the class if you like.
        List<int> moves = state.GetPossibleMoves();
        return moves[Random.Range(0, moves.Count)];
    }
}
