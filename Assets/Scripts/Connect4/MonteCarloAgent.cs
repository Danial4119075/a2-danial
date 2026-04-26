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
        int numMoves = moves.Count;

        // Split simulations evenly across all available moves
        int simsPerMove = totalSims / numMoves;

        float[] scores = new float[numMoves];

        for (int i = 0; i < numMoves; i++)
        {
            float totalScore = 0f;

            for (int s = 0; s < simsPerMove; s++)
            {
                // Clone the state so we don't affect the real game
                Connect4State simState = state.Clone();

                // Make the candidate move
                simState.MakeMove(moves[i]);

                // Simulate the rest of the game with random moves
                Connect4State.Result result = simState.GetResult();
                while (result == Connect4State.Result.Undecided)
                {
                    List<int> simMoves = simState.GetPossibleMoves();
                    int randomMove = simMoves[Random.Range(0, simMoves.Count)];
                    simState.MakeMove(randomMove);
                    result = simState.GetResult();
                }

                totalScore += Connect4State.ResultToFloat(result);
            }

            scores[i] = totalScore / simsPerMove;
        }

        // Choose the best move based on our player index
        // playerIdx 0 = Yellow (wants score close to 0), playerIdx 1 = Red (wants score close to 1)
        int bestMoveIdx = (playerIdx == 0) ? argMin(scores) : argMax(scores);
        return moves[bestMoveIdx];
    }
}