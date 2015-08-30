package cz.cvut.fel.oi.pah.uncertainty.viter;

import java.util.HashSet;
import java.util.Set;

import javax.vecmath.Point2i;


public class WorldModel {
	/**
	 * Returns a set of all transitions that may occur if 
	 * the given action is performed in the given s.
	 * 
	 * @param state the s in which is the action applied
	 * @param action the action applied
	 * @param map the map of the environment
	 * @return the set of transition, where is transition is assigned a probability, next s and the reward received.
	 */
	public static Set<Transition> getTransitions(Point2i state, Action action, CellContent[][] map) {
		Set<Transition> transitions = new HashSet<>();
		
		if (map[state.x][state.y] == CellContent.EMPTY) {
			
			switch (action) {
				case NORTH:
				{
					float prob = 0.8f;
					Point2i succState = nextState(state, Action.NORTH, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.EAST, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.WEST, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				break;
				
				case SOUTH:
				{
					float prob = 0.8f;
					Point2i succState = nextState(state, Action.SOUTH, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.EAST, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.WEST, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				break;
				
				case EAST:
				{
					float prob = 0.8f;
					Point2i succState = nextState(state, Action.EAST, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.NORTH, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.SOUTH, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				break;
				
				case WEST:
				{
					float prob = 0.8f;
					Point2i succState = nextState(state, Action.WEST, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.NORTH, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}
				{
					float prob = 0.1f;
					Point2i succState = nextState(state, Action.SOUTH, map);
					float reward = reward(state, succState, map);
					transitions.add(new Transition(prob, succState, reward));
				}	
				break;
			}
		} else {
			transitions.add(new Transition(1f, state, 0));
		}
		
		return transitions;
	}
	
	private static float reward(Point2i fromState, Point2i toState, CellContent[][] map) {

		if (map[toState.x][toState.y] == CellContent.GOLD) {
			return 100-1;
		}

		if (map[toState.x][toState.y] == CellContent.PIT) {
			return -100-1;
		}

		if (map[toState.x][toState.y] == CellContent.EMPTY) {
			return -1;
		}

		if (map[toState.x][toState.y] == CellContent.OBSTACLE) {
			return -1;
		}

		throw new RuntimeException();
	}



	private static Point2i nextState(Point2i state, Action action, CellContent[][] map) {
		Point2i desiredTarget;
		Point2i actualTarget;

		switch(action) {
			case NORTH:
				desiredTarget = new Point2i(state.x,state.y-1);
				break;

			case SOUTH:
				desiredTarget = new Point2i(state.x,state.y+1);
				break;

			case WEST:
				desiredTarget = new Point2i(state.x-1,state.y);
				break;

			case EAST:
				desiredTarget = new Point2i(state.x+1,state.y);
				break;
			default:
				throw new RuntimeException();
		}

		if (desiredTarget.x < 0 || desiredTarget.x >= map.length || desiredTarget.y < 0 || desiredTarget.y >= map[0].length ||  map[desiredTarget.x][desiredTarget.y] == CellContent.OBSTACLE) {
			actualTarget = new Point2i(state.x,state.y);
		} else {
			actualTarget = desiredTarget;
		}

		return actualTarget;
	}
}
