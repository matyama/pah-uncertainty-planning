package cz.cvut.fel.oi.pah.uncertainty.viter;

import java.awt.Color;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import tt.euclid2i.Point;
import tt.euclid2i.region.Circle;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.terminal.ArrowLayer;

public class ValueIterationAgentCreator {

	final static int CELLSIZE = 10;
	final static int COLS = 20;
	final static int ROWS = 20;
	final static double OBST_RATIO = 0.2;
	final static int STEPS = 200;
	private static final double PIT_RATIO = 0.05;

	CellContent[][] map; // First dimension is columns, second dimension is rows
	int x;
	int y;
	
	int lastX;
	int lastY;
	Action lastAction = null;

    private void initVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768, 200, 200);
        VisManager.setSceneParam(new SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(0, 0);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 5;
            }
        });

        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // Draw robot
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

			@Override
			public Collection<? extends tt.euclid2i.Region> getRegions() {
				Collection<Circle> regions = new LinkedList<Circle>();

				regions.add(new Circle(new Point(x*CELLSIZE, y*CELLSIZE),	CELLSIZE/2-2));

				return regions;
			}
		}, Color.BLACK, Color.BLUE));

        // Draw robot
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

			@Override
			public Collection<? extends tt.euclid2i.Region> getRegions() {
				return Collections.singleton(new Rectangle(new Point(-CELLSIZE/2, -CELLSIZE/2), 
						new Point(COLS * CELLSIZE - CELLSIZE/2, ROWS * CELLSIZE - CELLSIZE/2)));
			}
		}, Color.BLACK));

        // Draw obstacles
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

			@Override
			public Collection<? extends tt.euclid2i.Region> getRegions() {
				Collection<Rectangle> regions = new LinkedList<Rectangle>();
		    	for (int x=0; x < COLS; x++) {
		    		for (int y=0; y < ROWS; y++){
		    			if (map[x][y] == CellContent.OBSTACLE) {
		    				regions.add(new Rectangle(
		    						new Point(x*CELLSIZE - CELLSIZE/2, y*CELLSIZE - CELLSIZE/2),
		    						new Point(x*CELLSIZE + CELLSIZE/2, y*CELLSIZE + CELLSIZE/2)
		    				));
		    			}

		    		}
		    	}
				return regions;
			}
		}, Color.BLACK, Color.GRAY));

        // Draw gold
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

			@Override
			public Collection<? extends tt.euclid2i.Region> getRegions() {
				Collection<Circle> regions = new LinkedList<Circle>();
				for (int x=0; x < COLS; x++) {
		    		for (int y=0; y < ROWS; y++){
		    			if (map[x][y] == CellContent.GOLD) {
		    				regions.add(new Circle(new Point(x*CELLSIZE, y*CELLSIZE), CELLSIZE/2));
		    			}
		    		}
		    	}
				return regions;
			}
		}, Color.BLACK, Color.YELLOW));

        // Draw pit
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

			@Override
			public Collection<? extends tt.euclid2i.Region> getRegions() {
				Collection<Circle> regions = new LinkedList<Circle>();
				for (int x=0; x < COLS; x++) {
		    		for (int y=0; y < ROWS; y++){
		    			if (map[x][y] == CellContent.PIT) {
		    				regions.add(new Circle(new Point(x*CELLSIZE, y*CELLSIZE), CELLSIZE/2));
		    			}

		    		}
		    	}
				return regions;
			}
		}, Color.BLACK, Color.BLACK));
        
        
        // Draw last action
        VisManager.registerLayer(ArrowLayer.create(new LineElements() {
			
			@Override
			public int getStrokeWidth() {
				return 1;
			}
			
			@Override
			public Color getColor() {
				return Color.RED;
			}
			
			@Override
			public Iterable<? extends Line> getLines() {
				
				if (lastAction != null) {
					final Action action = lastAction;
					return Collections.singleton(new Line() {
						
						@Override
						public Point3d getTo() {
							int newX = lastX;
							int newY = lastY;
							
							switch (action) {
								case NORTH:	newY = lastY - 1; break;
								case SOUTH:	newY = lastY + 1; break;
								case WEST:	newX = lastX - 1; break;
								case EAST:	newX = lastX + 1; break;								
							}
							 
							return new Point3d(newX*CELLSIZE, newY*CELLSIZE, 0);
						}
						
						@Override
						public Point3d getFrom() {
							return new Point3d(lastX*CELLSIZE, lastY*CELLSIZE, 0);
						}
					});
				} else {
					return Collections.emptySet();
				}
			}
		}));
    }

    private void create() {
    	initVisualization();

    	final int RUNS = 10;
    	double sumUtility = 0;

    	for (int i=0; i<RUNS; i++) {
    		generateMap(new Random(3));
	    	double utility = simulate(new ValueIterationAgent(), new Random(i), 200);
	    	sumUtility += utility;
	    	System.out.println(">>> Simulation run finished. Utility: " + utility);
    	}
    	System.out.println(">>> " + RUNS + " simulation runs finished. Average utility: " + (sumUtility/RUNS) + "."  );
    }

    private double simulate(ValueIterationAgent agent, Random rnd, int visualizationDelay) {
    	x = 0;
    	y = 0;
    	double utility = 0;

    	for (int i=0; i<STEPS; i++) {
    		Action action = agent.nextStep(x,y,map);
    		
    		// visualize action execution
    		lastX = x;
    		lastY = y;
    		lastAction = null;
    		
    		try {
				Thread.sleep(visualizationDelay);
			} catch (InterruptedException e) {}
    		
    		lastAction = action;
    		
    		try {
				Thread.sleep(visualizationDelay);
			} catch (InterruptedException e) {}
    		
    		updatePosition(action, rnd);
    		utility -= 1.0; 

    		if (map[x][y] == CellContent.GOLD) {
    			System.out.println("Your robot found gold.");
    			utility += 100;
    			return utility;
    		}

    		if (map[x][y] == CellContent.PIT) {
    			System.out.println("Your robot fell into a pit.");
    			utility -= 100;
    			return utility;
    		}

    	}
    	return utility;
	}



	private void updatePosition(Action action, Random rnd) {
		int newX = x;
		int newY = y;
		
		if (action == Action.NORTH) {
			double r = rnd.nextDouble();
			if (r < 0.8)
				newY -= 1;
			else if (r < 0.9) {
				newX -= 1;
			} else {
				newX += 1;
			}
		}

		if (action == Action.SOUTH) {
			double r = rnd.nextDouble();
			if (r < 0.8)
				newY += 1;
			else if (r < 0.9) {
				newX -= 1;
			} else {
				newX += 1;
			}
		}

		if (action == Action.EAST) {
			double r = rnd.nextDouble();
			if (r < 0.8)
				newX += 1;
			else if (r < 0.9) {
				newY -= 1;
			} else {
				newY += 1;
			}
		}

		if (action == Action.WEST) {
			double r = rnd.nextDouble();
			if (r < 0.8)
				newX -= 1;
			else if (r < 0.9) {
				newY -= 1;
			} else {
				newY += 1;
			}
		}

		if (newX >= 0 && newX < COLS && newY >= 0 && newY < ROWS && map[newX][newY] != CellContent.OBSTACLE) {
			x = newX;
			y = newY;
		}

	}

	private void generateMap(Random rnd) {
    	x = 0;
    	y = 0;

    	map = new CellContent[COLS][ROWS];

    	for (int x=0; x < COLS; x++) {
    		for (int y=0; y < ROWS; y++){
    			map[x][y] = CellContent.EMPTY;
    		}
    	}

    	map[13][12] = CellContent.GOLD;


    	for (int x=0; x < COLS; x++) {
    		for (int y=0; y < ROWS; y++){
    			if (map[x][y] == CellContent.EMPTY) {
	    			if (rnd.nextDouble() < OBST_RATIO) {
	    				map[x][y] = CellContent.OBSTACLE;
	    			}
    			}
    		}
    	}

    	for (int x=0; x < COLS; x++) {
    		for (int y=0; y < ROWS; y++){
    			if (map[x][y] == CellContent.EMPTY) {
	    			if (rnd.nextDouble() < PIT_RATIO) {
	    				map[x][y] = CellContent.PIT;
	    			}
    			}
    		}
    	}

	}

	public static void main(String[] args) {
        new ValueIterationAgentCreator().create();
    }

}
