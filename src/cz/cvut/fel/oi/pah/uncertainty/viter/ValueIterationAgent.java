package cz.cvut.fel.oi.pah.uncertainty.viter;

import javax.vecmath.Point2i;

/**
 *
 * general value iteration (rewards depend not only on current s, but on the whole transition):
 *
 * V_0(s) = 0 for all s in S
 * V_{i+1}(s) = max_{a in A(s)} Q(s, a)
 *
 * Q(s, a) =def= sum_{s' in S} T_i(s, a, s') [R_i(s, a, s') + gamma * V_i(s')]
 *
 */
public class ValueIterationAgent {

    private static final double GAMMA = 0.9;
    private static final double EPSILON = 0.001;

    private static final int MAX_ITERS = 10000;
	
	private Action[][] policy;

	/**
	 * This method is called when the simulation engine requests the next action.
	 * You are given a position of the robot and the map of the environment.
	 *
	 * The top-left corner has coordinates (0,0). 
	 * 
	 * You can check whether there is an obstacle on a particular cell of the map 
	 * by querying map[x][y] == CellContent.OBSTACLE.
	 *
	 * There is one gold on the map. You can query whether a position contains gold by
	 * querying map[x][y] == CellContent.GOLD.
	 * 
	 * Further, there are several pits on the map. You can query whether a position contains pit by
	 * querying map[x][y] == CellContent.PIT.
	 *
	 * @param x the x-coordinate of the current position of robot
	 * @param y the y-coordinate of the current position of robot
	 * @param map the map of the environment
	 * @return action to perform in the next step
	 */
	public Action nextStep(int x, int y, CellContent[][] map) {

		if (policy == null) {
			DebugVis.initVis();
			// when called for the first time, compute the policy
			policy = computePolicy(map);
		}

		return policy[x][y];
	}

	/**
	 * Compute an optimal policy for the agent.
	 * @param map map of the environment
	 * @return an array that contains for each cell of the environment one action,
	 * i.e. one of: Action.NORTH, Action.SOUTH, Action.EAST, Action.WEST.
	 */
	private Action[][] computePolicy(CellContent[][] map) {

        int cols = map.length; // x-limit
        int rows = map[0].length; // y-limit

        float[][] U;
		float[][] newU = new float[cols][rows];
        Action[][] policy = new Action[cols][rows];

        int i = 0;
        double delta;

        do {

            i++;
            delta = 0.0;

            U = newU;
            newU = new float[cols][rows];

            // for each state (x, y)
            for (int x = 0; x < cols; x++) { // x-axis
                for (int y = 0; y < rows; y++) { // y-axis

                    Point2i s = new Point2i(x, y);

                    float maxQ = Float.NEGATIVE_INFINITY;
                    Action maxAction = Action.NORTH;

                    for (Action a : Action.values()) {

                        float Q = 0f;
                        for (Transition t : WorldModel.getTransitions(s, a, map)) {
                            Q += t.probability * (t.reward + GAMMA * U[t.successorState.x][t.successorState.y]);
                        }

                        if (Q > maxQ) {
                            maxQ = Q;
                            maxAction = a;
                        }

                    }

                    newU[x][y] = maxQ;
                    policy[x][y] = maxAction;

                    double diff = Math.abs(newU[x][y] - U[x][y]);
                    if (diff > delta) {
                        delta = diff;
                    }

                }
            }

        } while (delta >= EPSILON * (1.0 - GAMMA) / GAMMA && i < MAX_ITERS);
		
		return policy;
	}
	
}
