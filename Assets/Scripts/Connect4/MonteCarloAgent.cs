using UnityEngine;
using System.Collections.Generic;

public class MonteCarloAgent : Agent
{
    public int totalSims = 2500;

    public override int GetMove(Connect4State state)
    {
        // TODO: Override this method with the logic described in class.
        // Currently, it just returns a random move.
        // You can add other methods to the class if you like.
        List<int> moves = state.GetPossibleMoves();
        return moves[Random.Range(0, moves.Count)];
    }
}
