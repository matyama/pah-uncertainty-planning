package cz.cvut.fel.oi.pah.uncertainty.viter;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

import javax.vecmath.Matrix4d;
import javax.vecmath.Tuple2i;
import javax.vecmath.Vector3d;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.TerminalLayer;


public class PolicyLayer extends TerminalLayer {

    static public interface PolicyProvider {
       Action[][] getPolicy();
    }

    PolicyProvider provider;
	private int cellSize;

    PolicyLayer(PolicyProvider provider, int cellSize) {
        this.provider = provider;
        this.cellSize = cellSize;
    }

    @Override
    public void paint(Graphics2D canvas) {
        canvas.setStroke(new BasicStroke(1));
        canvas.setColor(Color.BLUE);

        Action[][] policy = provider.getPolicy();
        
        if (policy != null) {
        
        int cols = policy.length;
        int rows = policy[0].length;
	        for (int x = 0; x < cols; x++) {
	        	for (int y = 0; y < rows; y++) {
	        		if (policy[x][y] != null) {
		        		Tuple2i dir = policy[x][y].getDirection();	        		
		        		drawArrow(canvas, x*cellSize, y*cellSize, x*cellSize + dir.x*cellSize/4, y*cellSize + dir.y*cellSize/4 );
	        		}
	        	}
				
			}
        }
    }
    
    protected void drawArrow(Graphics2D canvas, int x1, int y1, int x2, int y2) {
    	
        x1 = Vis.transX(x1);
        y1 = Vis.transY(y1);
        x2 = Vis.transX(x2);
        y2 = Vis.transY(y2);
    	
        Vector3d arrowPart1 = new Vector3d(x2 - x1, y2 - y1, 0);
        Vector3d arrowPart2 = new Vector3d(arrowPart1);
        Matrix4d transf1 = new Matrix4d();
        transf1.rotZ(5 * Math.PI / 6);
        transf1.transform(arrowPart1);
        transf1.rotZ(7 * Math.PI / 6);
        transf1.transform(arrowPart2);
        arrowPart1.normalize();
        arrowPart1.scale(5);

        arrowPart2.normalize();
        arrowPart2.scale(5);
        
        canvas.drawLine(x1, y1, x2, y2);
        canvas.drawLine(x2, y2, x2 + (int) arrowPart1.x, y2 + (int) arrowPart1.y);
        canvas.drawLine(x2, y2, x2 + (int) arrowPart2.x, y2 + (int) arrowPart2.y);
    }

    public static VisLayer create(PolicyProvider provider, int cellSize) {
        return new PolicyLayer(provider,cellSize);
    }
}
