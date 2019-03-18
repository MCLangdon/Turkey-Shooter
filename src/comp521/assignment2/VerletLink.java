package comp521.assignment2;

public class VerletLink {

    private VerletPoint pointA;
    private VerletPoint pointB;
    private double restingDistance;

    public VerletLink(VerletPoint a, VerletPoint b, double distance){
        pointA = a;
        pointB = b;
        restingDistance = distance;
    }

    public void resolveConstraints(){
        // Determine a scalar of the difference between the current distance between points A and B and the resting distance of the link.
        double distX = pointA.getCurrent().getX() - pointB.getCurrent().getX();
        double distY = pointA.getCurrent().getY() - pointB.getCurrent().getY();
        double dist = Math.sqrt((distX*distX) + (distY*distY));
        double difference = (restingDistance - dist) / dist;

        // Use this scalar to determine how much to move points A and B.
        // Move points halfway towards their resting distance.
        double moveX = distX * 0.5 * difference;
        double moveY = distY * 0.5 * difference;
        pointA.getCurrent().setLocation(pointA.getCurrent().getX() + moveX, pointA.getCurrent().getY() + moveY);
        pointB.getCurrent().setLocation(pointB.getCurrent().getX() - moveX, pointB.getCurrent().getY() - moveY);
    }
}
