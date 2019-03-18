package comp521.assignment2;

public class Cloud {

    private double xCoord;
    private double yCoord;

    public Cloud(double x, double y){
        xCoord = x;
        yCoord = y;
    }

    public double getXCoord(){
        return xCoord;
    }

    public double getYCoord(){
        return yCoord;
    }

    public void updateLocation(double windVelocity, float timestep){
        // Wind acceleration only occurs along x axis, so yCoord never changes.
        xCoord = xCoord + windVelocity*timestep;
    }
}
