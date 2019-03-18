package comp521.assignment2;

public class Canon {
    private int xCoord; // x-coordinate of center of rotation of canon
    private int yCoord; // y-coordinate of center of rotation of canon
    private double theta; // angle of canon in degrees, restricted to [280, 360]

    public Canon(int x, int y, double angle){
        xCoord = x;
        yCoord = y;
        theta = angle;
    }

    public double getAngle(){
        return theta;
    }

    public int getXCoord(){
        return xCoord;
    }

    public int getYCoord(){
        return yCoord;
    }

    public void incrementAngle(){
        if (theta < 360){
            theta++;
        }
    }

    public void decrementAngle(){
        if (theta > 280){
            theta--;
        }
    }
}
