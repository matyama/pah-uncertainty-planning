package cz.cvut.fel.oi.pah.uncertainty.viter;

import javax.vecmath.Point2i;


public class Transition {
	public Transition(float probability, Point2i successorState,
			float reward) {
		super();
		this.probability = probability;
		this.successorState = successorState;
		this.reward = reward;
	}
	
	final float probability;
	final Point2i successorState;
	final float reward;
}