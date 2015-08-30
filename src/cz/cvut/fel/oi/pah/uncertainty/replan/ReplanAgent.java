package cz.cvut.fel.oi.pah.uncertainty.replan;

import javax.vecmath.Point2i;
import java.util.*;

/**
 * replan agent (uses enforced hill-climbing with h = manhattan distance)
 *
 * @author matyama
 */
public class ReplanAgent {

	private List<Action> plan;
	
	private Point2i expected;

    private CellContent[][] map;
	private Point2i goal;
	
	/**
	 * This method is called after when a simulation engine request next action.
	 * You are given a position of the robot and the map of the environment.
	 *
	 * The top-left corner has coordinates (0,0). You can check whether
	 * there is an obstacle by querying map[x][y] == CellContent.Obstacle.
	 *
	 * There is one gold on the map. You can query whether a position contains gold by
	 * querying map[x][y] == CellContent.Gold.
	 *
	 * @param x the x-coordinate of the current position of robot
	 * @param y the y-coordinate of the current position of robot
	 * @param map the map of the environment
	 * @return action to perform in the next step
	 */
	public Action nextStep(int x, int y, CellContent[][] map) {

        if (plan == null) {
            this.map = map;
            goal = extractGoal(map);
        }

        Point2i actual = new Point2i(x, y);

        if (!actual.equals(expected)) {
            plan = plan(actual);
        }

        if (plan == null || plan.isEmpty()) {
            throw new NoSuchElementException("cannot perform next step: plan does not exist or is empty");
        }

        Action action = plan.remove(0);
        expected = action.apply(actual);
        return action;
	}
	
	private List<Action> plan(Point2i init) {
        Node n = new Node(null, null, init);
        while (!n.isGoal()) {
            n = improve(n);
        }
        return n.extractSolution();
	}

    private Node improve(Node init) {

        Queue<Node> queue = new LinkedList<>();
        queue.add(init);
        Set<Point2i> closed = new HashSet<>();

        while (!queue.isEmpty()) {
            Node n = queue.poll();
            if (!closed.contains(n.getState())) {
                closed.add(n.getState());
                if (h(n) < h(init)) {
                    return n;
                }
                for (Transition t : n.succ()) {
                    queue.add(new Node(n, t.getOperator(), t.getState()));
                }
            }
        }

        return new Fail();
    }
	
	private int h(Node n) {
		return Math.abs(n.getState().x - goal.x) + Math.abs(n.getState().y - goal.y);
	}
	
	private Point2i extractGoal(CellContent[][] map) {
		for (int x = 0; x < map.length; x++) {
			for (int y = 0; y < map[0].length; y++) {
				if (map[x][y] == CellContent.GOLD) {
                    return new Point2i(x, y);
                }
			}
		}
		throw new NoSuchElementException("global goal (gold) not found on the map");
	}
	
	private class Node {

        private final Node parent;
        private final Action operator;
		private final Point2i state;
		
		public Node(Node parent, Action operator, Point2i state) {
            this.parent = parent;
            this.operator = operator;
			this.state = state;
		}

        public Action getOperator() {
            return operator;
        }

        public Point2i getState() {
            return state;
        }

        public boolean isGoal() {
            return goal.equals(state);
        }

        private boolean safe(Point2i s) {
            return s.x >= 0 && s.x < map.length && s.y >= 0 && s.y < map[0].length &&
                    map[s.x][s.y] != CellContent.OBSTACLE && map[s.x][s.y] != CellContent.PIT;
        }

        public Iterable<Transition> succ() {
            List<Transition> transitions = new ArrayList<>(Action.values().length);
            for (Action op : Action.values()) {
                Point2i s = op.apply(state);
                if (safe(s)) {
                    transitions.add(new Transition(op, s));
                }
            }
            return transitions;
        }

        public List<Action> extractSolution() {
            LinkedList<Action> solution = new LinkedList<>();
            Node n = this;
            while (n.parent != null) {
                solution.addFirst(n.getOperator());
                n = n.parent;
            }
            return solution;
        }
		
	}

    private class Fail extends Node {

        public Fail() {
            super(null, null, null);
        }

        @Override
        public boolean isGoal() {
            return true;
        }

        @Override
        public List<Action> extractSolution() {
            return new LinkedList<>();
        }

    }

    private class Transition {

        private final Action operator;
        private final Point2i state;

        public Transition(Action operator, Point2i state) {
            this.operator = operator;
            this.state = state;
        }

        public Action getOperator() {
            return operator;
        }

        public Point2i getState() {
            return state;
        }

    }
	
}
