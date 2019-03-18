package comp521.assignment2;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class CanonBall {

    private int mass = 5;
    private int radius;
    private double xCoord; // x-coordinate of canonball location used for drawing
    private double yCoord; // y-coordinate of canonball location used for drawing
    private boolean bouncing;

    private Polygon mountain;
    private int groundlevel;
    private ArrayList<Turkey> turkeys;
    private int mountainLeftBase;
    private int mountainRightBase;

    private double xVelocity;
    private double yVelocity;

    // Coordinatees of center of canonball, used for detecting collision.
    private double xCenter;
    private double yCenter;

    private boolean inMotion;
    private long flightStartTime;
    private long flightEndTime;
    private boolean active;
    private int bounces;

    private int timer;

    public CanonBall(int radius, double xCoord, double yCoord, double initialVelocity, double theta, Polygon mountain, int groundlevel, ArrayList<Turkey> turkeys, int mountainLLeftBase, int mountainRightBase){
        this.radius = radius;
        this.xCoord = xCoord;
        this.yCoord = yCoord;
        this.mountain = mountain;
        this.groundlevel = groundlevel;
        this.turkeys = turkeys;
        this.mountainLeftBase = mountainLLeftBase;
        this.mountainRightBase = mountainRightBase;

        this.xVelocity = initialVelocity*(Math.cos(Math.toRadians(theta)));
        this.yVelocity = initialVelocity*(Math.sin(Math.toRadians(theta)));
        this.xCenter = xCoord + radius;
        this.yCenter = yCoord + radius;
        this.inMotion = true;
        flightStartTime = System.nanoTime();
        active = true;
        bounces = 0;
        timer = 0;

    }

    public double getxCoord(){
        return xCoord;
    }

    public double getyCoord(){
        return yCoord;
    }

    public double getYCenter(){
        return yCenter;
    }

    public void updateVelocity(double windForce, double gravity, float timestep){
        if(inMotion) {
            // Apply acceleration of gravity to yVelocity.
            yVelocity = yVelocity + gravity * timestep;

            // Apply acceleration of wind to xVelocity (assuming wind acceleration is only along x-axis).
            // acceleration of wind = force/mass.
            xVelocity = xVelocity + (windForce/mass);
        }
    }

    public void updateLocation(float timestep){
        xCoord = xCoord + xVelocity * timestep;
        yCoord = yCoord + yVelocity * timestep;
        xCenter = xCoord + radius;
        yCenter = yCoord + radius;

        // Check for collision with wall. If colliding with wall, reverse xVelocity and reduce by coefficient of
        // restitution = 0.70. yVelocity does not change.
        if (xCoord <= 15) {
            xVelocity = -0.70 * xVelocity;
        }
        // Check for collision with ground. Stop canonball if it is colliding with the ground
        else if(yCenter > groundlevel){
            stop(groundlevel);
        }

        // Check for collision with mountain.
        else if (xCenter >= mountainLeftBase && xCenter <= mountainRightBase) {
            // Determine two points on mountain closest to xCoord.
            int point1Index = (int) (xCenter - mountainLeftBase) / 5;
            int point2Index = point1Index + 1;

            // Determine the distance between the center of canonball and the line segment created by the mountain points.
            Line2D segment = new Line2D.Double(point1Index * 5 + mountainLeftBase, mountain.ypoints[point1Index], point2Index * 5 + mountainLeftBase, mountain.ypoints[point2Index]);
            double distance = segment.ptLineDist(xCenter, yCenter);

            // Determine the height of the higher of the two points.
            int maxHeight = mountain.ypoints[point1Index];
            if (mountain.ypoints[point2Index] < maxHeight) {
                maxHeight = mountain.ypoints[point2Index];
            }

            // If the distance is <= the radius of the canonball, a collision is occurring. Relocate canonball to last position before
            // collision occurred and modify its velocity.
            if (distance <= radius && !bouncing) {
                xCoord = xCoord - xVelocity * timestep;
                yCoord = yCoord - yVelocity * timestep;

                // If canonball has already bounced off of the mountain twice, it is stuck between peaks so halt it.
                if (bounces > 2){
                    stop((int) yCenter);
                }
                // Determine direction of bounce.
                else if (xCenter <= (mountainLeftBase + mountainRightBase) / 2) {
                    // Bounce up to the left with coefficient of restitution = 0.70.
                    xVelocity = 0.70 * xVelocity;
                    yVelocity = -0.70 * yVelocity;
                } else {
                    // Bounce up to the right with coefficient of restitution = 0.70.
                    xVelocity = -0.70 * xVelocity;
                    yVelocity = -0.70 * yVelocity;
                }
            }
            // If the distance is > the radius of the canonball but is less than the higher of the two points, the canonball is entirely within the
            // mountain, so move it back to the last position outside of the mountain and bounce it.
            else if (yCenter > maxHeight && !bouncing) {
                xCoord = xCoord - xVelocity * timestep;
                yCoord = yCoord - yVelocity * timestep;
                xCenter = xCoord + radius;
                yCenter = yCenter + radius;
                bouncing = true;

                // If canonball has already bounced off of the mountain twice, it is stuck between peaks so halt it.
                bounces++;
                if (bounces > 2){
                    stop((int) yCenter);
                }
                // Determine direction of bounce.
                else if (xCenter <= (mountainLeftBase + mountainRightBase) / 2) {
                    // Bounce up to the left with coefficient of restitution = 0.70.
                    xVelocity = 0.70 * xVelocity;
                    yVelocity = -0.70 * yVelocity;
                } else {
                    // Bounce up to the right with coefficient of restitution = 0.70.
                    xVelocity = -0.70 * xVelocity;
                    yVelocity = -0.70 * yVelocity;
                }
            } else {
                bouncing = false;
            }
        }

        // Check for collision with a turkey by checking if line segment of canonball between previous position and current position
        // intersects an edge of a turkey.
        else {
            for (Turkey t : turkeys) {
                for (TurkeyEdge e : t.getEdges()){
                    Line2D edge = e.getLineSegment();
                    Line2D trajectory = new Line2D.Double(xCenter, yCenter, xCenter - xVelocity, yCenter - yVelocity);
                    boolean intersected = trajectory.intersectsLine(edge);
                    // If canonball collided with an edge of the turkey, apply the force of the canonball to the verlet points of that edge
                    // and deactive the canonball.
                    if (intersected){
                        VerletPoint a = e.getA();
                        VerletPoint b = e.getB();
                        a.addXForce(xVelocity);
                        a.addYForce(yVelocity);
                        b.addXForce(xVelocity);
                        b.addYForce(yVelocity);

                        active = false;
                    }
                }
            }
        }

        // Deactive canonball after a period of time.
        timer ++;
        if (timer > 100){
            active = false;
        }
    }

    public void stop(int yPosition){
        // Stop canonball at indicated y position.
        yCenter = yPosition;
        yCoord = yCenter - radius;

        // If this is the first time stop() is being called on this canonball, it has just collided with the ground, so
        // set the flightEndTime, inMotion, and velocities.
        if (xVelocity != 0 && yVelocity != 0) {
            flightEndTime = System.nanoTime();
            inMotion = false;
            xVelocity = 0;
            yVelocity = 0;
        }

    }

    public long getFlightTime(){
        return flightEndTime - flightStartTime;
    }

    public boolean isActive(){
        return active;
    }
}
