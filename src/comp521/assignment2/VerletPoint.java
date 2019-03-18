package comp521.assignment2;

import java.awt.geom.Point2D;

public class VerletPoint {

    private Point2D.Double initialPosition;
    private Point2D.Double currentPosition;
    private Point2D.Double oldPosition;
    private double gravity;
    private double xForce; // Force applied to point in the x-direction.
    private double yForce; // Force applied to point in the y-direction.

    public VerletPoint(Point2D.Double current, double xForce, double gravity){
        initialPosition = current;
        currentPosition = current;
        oldPosition = current;
        this.xForce = xForce;
        this.gravity = gravity;

        // Turkey points will always be initialized with gravity applied.
        yForce = gravity;
    }

    public Point2D.Double getOld(){
        return oldPosition;
    }

    public Point2D.Double getCurrent(){
        return currentPosition;
    }

    public double getXForce(){
        return xForce;
    }

    public double getYForce(){
        return yForce;
    }

    public void addXForce(double force){
        xForce = xForce + force;
    }

    public void addYForce(double force){
        yForce = yForce + force;
    }

    public void setCurrentPosition(Point2D.Double current){
        currentPosition = current;
    }

    public void setOldPosition(Point2D.Double old){
        oldPosition = old;
    }

    public void reset(double paceVelocity){
        currentPosition = initialPosition;
        oldPosition = initialPosition;
        xForce = paceVelocity;
        yForce = gravity;
    }
}
