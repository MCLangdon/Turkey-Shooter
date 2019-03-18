package comp521.assignment2;

import java.awt.geom.Line2D;

public class TurkeyEdge {

    private Line2D lineSegment;
    private VerletPoint a;
    private VerletPoint b;

    public TurkeyEdge(VerletPoint a, VerletPoint b){
        this.a = a;
        this.b = b;
        lineSegment = new Line2D.Double(a.getCurrent().getX(), a.getCurrent().getY(), b.getCurrent().getX(), b.getCurrent().getY());
    }

    public Line2D getLineSegment(){
        return lineSegment;
    }

    public VerletPoint getA(){
        return a;
    }

    public VerletPoint getB(){
        return b;
    }
}
