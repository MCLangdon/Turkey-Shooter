package comp521.assignment2;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Random;

public class Turkey {

    private ArrayList<TurkeyEdge> edges;
    private ArrayList<VerletPoint> points;
    private ArrayList<VerletLink> links;

    private int xOffset;
    private int groundlevel;
    private int leftMountainBase;
    private int xMountainPeak; // x coordinate of the peak of the mountain (center of the window);
    private int yMountainPeak; // y coordinate of the peak of the mountain.
    private double paceSpeed;
    private Polygon mountain;
    private int mass;
    private boolean isGrounded;
    private double jumpForce = -900;

    private double gravity;


    public Turkey(int xOffset, double paceSpeed, Polygon mountain,int groundlevel, int leftMountainBase, int xMountainPeak, int yMountainPeak, double gravity){
        this.xOffset = xOffset;
        this.paceSpeed = paceSpeed;
        this.mountain = mountain;
        this.groundlevel = groundlevel;
        this.leftMountainBase = leftMountainBase;
        this.xMountainPeak = xMountainPeak;
        this.yMountainPeak = yMountainPeak;
        this.gravity = gravity;

        edges = new ArrayList<TurkeyEdge>();
        points = new ArrayList<VerletPoint>();
        links = new ArrayList<VerletLink>();
        isGrounded = true;
        mass = 5;

        // Randomly choose x-direction for turkey to pace.
        double paceVelocity;
        Random rand = new Random();
        double paceDirection = rand.nextDouble();
        // If paceDirection <= 0.5, pace to the left. Otherwise, pace to the right.
        if (paceDirection <= 0.5){
            paceVelocity = -1*paceSpeed;
        } else {
            paceVelocity = paceSpeed;
        }

        // Initialize and add verlet points.
        VerletPoint a = new VerletPoint(new Point2D.Double(20 + xOffset, groundlevel - 30), paceVelocity, this.gravity);
        VerletPoint b = new VerletPoint(new Point2D.Double(25 + xOffset, groundlevel - 35), paceVelocity, this.gravity);
        VerletPoint c = new VerletPoint(new Point2D.Double(30 + xOffset, groundlevel - 35), paceVelocity, this.gravity);
        VerletPoint d = new VerletPoint(new Point2D.Double(33 + xOffset, groundlevel - 25), paceVelocity, this.gravity);
        VerletPoint e = new VerletPoint(new Point2D.Double(43 + xOffset, groundlevel - 32), paceVelocity, this.gravity);
        VerletPoint f = new VerletPoint(new Point2D.Double(53 + xOffset, groundlevel - 32), paceVelocity, this.gravity);
        VerletPoint g = new VerletPoint(new Point2D.Double(58 + xOffset, groundlevel - 22), paceVelocity, this.gravity);
        VerletPoint h = new VerletPoint(new Point2D.Double(58 + xOffset, groundlevel - 16), paceVelocity, this.gravity);
        VerletPoint i = new VerletPoint(new Point2D.Double(53 + xOffset, groundlevel - 11), paceVelocity, this.gravity);
        VerletPoint j = new VerletPoint(new Point2D.Double(48 + xOffset, groundlevel - 8), paceVelocity, this.gravity);
        VerletPoint k = new VerletPoint(new Point2D.Double(48 + xOffset, groundlevel - 3), paceVelocity, this.gravity);
        VerletPoint l = new VerletPoint(new Point2D.Double(43 + xOffset, groundlevel), paceVelocity, this.gravity);
        VerletPoint m = new VerletPoint(new Point2D.Double(38 + xOffset, groundlevel), paceVelocity, this.gravity);
        VerletPoint n = new VerletPoint(new Point2D.Double(43 + xOffset, groundlevel - 3), paceVelocity, this.gravity);
        VerletPoint o = new VerletPoint(new Point2D.Double(43 + xOffset, groundlevel - 8), paceVelocity, this.gravity);
        VerletPoint p = new VerletPoint(new Point2D.Double(38 + xOffset, groundlevel - 8), paceVelocity, this.gravity);
        VerletPoint q = new VerletPoint(new Point2D.Double(28 + xOffset, groundlevel - 16), paceVelocity, this.gravity);
        VerletPoint r = new VerletPoint(new Point2D.Double(28 + xOffset, groundlevel - 25), paceVelocity, this.gravity);
        VerletPoint s = new VerletPoint(new Point2D.Double(25 + xOffset, groundlevel - 20), paceVelocity, this.gravity);
        VerletPoint t = new VerletPoint(new Point2D.Double(25 + xOffset, groundlevel - 27), paceVelocity, this.gravity);
        VerletPoint u = new VerletPoint(new Point2D.Double(25 + xOffset, groundlevel -30) , paceVelocity, this.gravity);

        points.add(a);
        points.add(b);
        points.add(c);
        points.add(d);
        points.add(e);
        points.add(f);
        points.add(g);
        points.add(h);
        points.add(i);
        points.add(j);
        points.add(k);
        points.add(l);
        points.add(m);
        points.add(n);
        points.add(o);
        points.add(p);
        points.add(q);
        points.add(r);
        points.add(s);
        points.add(t);
        points.add(u);

        // Initialize and add verlet links, which will maintain the constraints of the turkey.
        // VerletLinks of drawn edges of turkey:
        VerletLink ab = new VerletLink(a, b, a.getCurrent().distance(b.getCurrent()));
        VerletLink bc = new VerletLink(b, c, b.getCurrent().distance(c.getCurrent()));
        VerletLink cd = new VerletLink(c, d, c.getCurrent().distance(d.getCurrent()));
        VerletLink de = new VerletLink(d, e, d.getCurrent().distance(e.getCurrent()));
        VerletLink ef = new VerletLink(e, f, e.getCurrent().distance(f.getCurrent()));
        VerletLink fg = new VerletLink(f, g, f.getCurrent().distance(g.getCurrent()));
        VerletLink gh = new VerletLink(g, h, g.getCurrent().distance(h.getCurrent()));
        VerletLink hi = new VerletLink(h, i, h.getCurrent().distance(i.getCurrent()));
        VerletLink ij = new VerletLink(i, j, i.getCurrent().distance(j.getCurrent()));
        VerletLink jk = new VerletLink(j, k, j.getCurrent().distance(k.getCurrent()));
        VerletLink kl = new VerletLink(k, l, k.getCurrent().distance(l.getCurrent()));
        VerletLink jo = new VerletLink(j, o, j.getCurrent().distance(o.getCurrent()));
        VerletLink on = new VerletLink(o, n, o.getCurrent().distance(n.getCurrent()));
        VerletLink nm = new VerletLink(n, m, n.getCurrent().distance(m.getCurrent()));
        VerletLink op = new VerletLink(o, p, o.getCurrent().distance(p.getCurrent()));
        VerletLink pq = new VerletLink(p, q, p.getCurrent().distance(q.getCurrent()));
        VerletLink qr = new VerletLink(q, r, q.getCurrent().distance(r.getCurrent()));
        VerletLink rs = new VerletLink(r, s, r.getCurrent().distance(s.getCurrent()));
        VerletLink st = new VerletLink(s, t, s.getCurrent().distance(t.getCurrent()));
        VerletLink ta = new VerletLink(t, a, t.getCurrent().distance(a.getCurrent()));
        // VerletLinks that are not drawn:
        VerletLink bt = new VerletLink(b, t, b.getCurrent().distance(t.getCurrent()));
        VerletLink cr = new VerletLink(c, r, c.getCurrent().distance(r.getCurrent()));
        VerletLink rt = new VerletLink(r, t, r.getCurrent().distance(t.getCurrent()));
        VerletLink dq = new VerletLink(d, q, d.getCurrent().distance(q.getCurrent()));
        VerletLink ce = new VerletLink(c, e, c.getCurrent().distance(e.getCurrent()));
        VerletLink dg = new VerletLink(d, g, d.getCurrent().distance(g.getCurrent()));
        VerletLink qh = new VerletLink(q, h, q.getCurrent().distance(h.getCurrent()));
        VerletLink ep = new VerletLink(e, p, e.getCurrent().distance(p.getCurrent()));
        VerletLink fi = new VerletLink(f, i, f.getCurrent().distance(i.getCurrent()));
        VerletLink sq = new VerletLink(s, q, s.getCurrent().distance(q.getCurrent()));
        VerletLink pm = new VerletLink(p, m, p.getCurrent().distance(m.getCurrent()));
        VerletLink ml = new VerletLink(m, l, m.getCurrent().distance(l.getCurrent()));
        VerletLink mq = new VerletLink(m, q, m.getCurrent().distance(q.getCurrent()));
        VerletLink ej = new VerletLink(e, j, e.getCurrent().distance(j.getCurrent()));
        VerletLink fj = new VerletLink(f, j, f.getCurrent().distance(j.getCurrent()));
        VerletLink jl = new VerletLink(j, l, j.getCurrent().distance(l.getCurrent()));
        VerletLink mo = new VerletLink(m, o, m.getCurrent().distance(o.getCurrent()));
        VerletLink dp = new VerletLink(d, p, d.getCurrent().distance(p.getCurrent()));
        VerletLink cm = new VerletLink(c, m, c.getCurrent().distance(m.getCurrent()));
        VerletLink el = new VerletLink(e, l, e.getCurrent().distance(l.getCurrent()));
        VerletLink gl = new VerletLink(g, l, g.getCurrent().distance(l.getCurrent()));
        VerletLink bl = new VerletLink(b, l, b.getCurrent().distance(l.getCurrent()));
        VerletLink bu = new VerletLink(b, u, b.getCurrent().distance(u.getCurrent()));
        VerletLink tu = new VerletLink(t, u, t.getCurrent().distance(u.getCurrent()));
        VerletLink du = new VerletLink(d, u, d.getCurrent().distance(u.getCurrent()));
        VerletLink od = new VerletLink(o, d, o.getCurrent().distance(d.getCurrent()));
        VerletLink bs = new VerletLink(b, s, b.getCurrent().distance(s.getCurrent()));
        VerletLink kh = new VerletLink(k, h, k.getCurrent().distance(h.getCurrent()));
        VerletLink em = new VerletLink(e, m, e.getCurrent().distance(m.getCurrent()));
        VerletLink dh = new VerletLink(d, h, d.getCurrent().distance(h.getCurrent()));
        VerletLink am = new VerletLink(a, m, a.getCurrent().distance(m.getCurrent()));
        VerletLink gn = new VerletLink(g, n, g.getCurrent().distance(n.getCurrent()));
        VerletLink ok = new VerletLink(o, k, o.getCurrent().distance(k.getCurrent()));
        VerletLink eh = new VerletLink(e, h, e.getCurrent().distance(h.getCurrent()));
        VerletLink rh = new VerletLink(r, h, r.getCurrent().distance(h.getCurrent()));
        VerletLink gq = new VerletLink(g, q, g.getCurrent().distance(q.getCurrent()));
        VerletLink oc = new VerletLink(o, c, o.getCurrent().distance(c.getCurrent()));
        VerletLink sd = new VerletLink(s, d, s.getCurrent().distance(d.getCurrent()));
        VerletLink gb = new VerletLink(g, b, g.getCurrent().distance(b.getCurrent()));
        VerletLink et = new VerletLink(e, t, e.getCurrent().distance(t.getCurrent()));
        VerletLink ae = new VerletLink(a, e, a.getCurrent().distance(e.getCurrent()));
        VerletLink sh = new VerletLink(s, h, s.getCurrent().distance(h.getCurrent()));
        VerletLink fp = new VerletLink(f, p, f.getCurrent().distance(p.getCurrent()));
        VerletLink fo = new VerletLink(f, o, f.getCurrent().distance(o.getCurrent()));
        VerletLink ki = new VerletLink(k, i, k.getCurrent().distance(i.getCurrent()));

        links.add(ab);
        links.add(bc);
        links.add(cd);
        links.add(de);
        links.add(ef);
        links.add(fg);
        links.add(gh);
        links.add(hi);
        links.add(ij);
        links.add(jk);
        links.add(kl);
        links.add(jo);
        links.add(on);
        links.add(nm);
        links.add(op);
        links.add(pq);
        links.add(qr);
        links.add(rs);
        links.add(st);
        links.add(ta);
        links.add(bt);
        links.add(cr);
        links.add(rt);
        links.add(dq);
        links.add(ce);
        links.add(dg);
        links.add(qh);
        links.add(ep);
        links.add(fi);
        links.add(sq);
        links.add(pm);
        links.add(ml);
        links.add(mq);
        links.add(ej);
        links.add(fj);
        links.add(jl);
        links.add(mo);
        links.add(dp);
        links.add(cm);
        links.add(el);
        links.add(gl);
        links.add(bl);
        links.add(bu);
        links.add(tu);
        links.add(du);
        links.add(od);
        links.add(bs);
        links.add(kh);
        links.add(em);
        links.add(dh);
        links.add(am);
        links.add(gn);
        links.add(ok);
        links.add(eh);
        links.add(rh);
        links.add(gq);
        links.add(oc);
        links.add(sd);
        links.add(gb);
        links.add(et);
        links.add(ae);
        links.add(sh);
        links.add(fp);
        links.add(fo);
        links.add(ki);

        // Initialize and add edges of turkey that will be drawn.
        edges.add(new TurkeyEdge(a, b));
        edges.add(new TurkeyEdge(b, c));
        edges.add(new TurkeyEdge(c, d));
        edges.add(new TurkeyEdge(d, e));
        edges.add(new TurkeyEdge(e, f));
        edges.add(new TurkeyEdge(f, g));
        edges.add(new TurkeyEdge(g, h));
        edges.add(new TurkeyEdge(h, i));
        edges.add(new TurkeyEdge(i, j));
        edges.add(new TurkeyEdge(j, k));
        edges.add(new TurkeyEdge(k, l));
        edges.add(new TurkeyEdge(j, o));
        edges.add(new TurkeyEdge(o, n));
        edges.add(new TurkeyEdge(n, m));
        edges.add(new TurkeyEdge(o, p));
        edges.add(new TurkeyEdge(p, q));
        edges.add(new TurkeyEdge(q, r));
        edges.add(new TurkeyEdge(r, s));
        edges.add(new TurkeyEdge(s, t));
        edges.add(new TurkeyEdge(t, a));
    }

    public void updateLocation(double windVelocity, double gravity, float timestep, Polygon mountain){
        double newX;
        double newY;

        for (VerletPoint p : points) {
            Point2D.Double old = p.getOld();
            Point2D.Double current = p.getCurrent();

            newX = current.getX() + (current.getX() - old.getX() + p.getXForce()) * timestep;
            newY = current.getY() + (current.getY() - old.getY() + p.getYForce()) * timestep;

            if (isGrounded) {
                // If turkey is already on ground, apply an equal and opposite impulse
                // to simulate turkey "standing up," effectively cancelling effect of gravity.

                p.addYForce(-1 * p.getYForce());
                newY = current.getY() + (current.getY() - old.getY() + p.getYForce()) * timestep;

                p.setOldPosition(current);
                p.setCurrentPosition(new Point2D.Double(newX, newY));
                p.addYForce(gravity * timestep);
            }
            else {
                // Turkey is in the air. Check if turkey is above mountain peak.
                if (current.getY() > yMountainPeak) {
                    // Apply force of wind.
                    p.addXForce(windVelocity);
                    newX = current.getX() + (current.getX() - old.getX() + p.getXForce()) * timestep;
                    p.addXForce(-1 * windVelocity);
                }

                newY = current.getY() + (current.getY() - old.getY() + p.getYForce()) * timestep;
                p.addYForce(gravity * timestep);
                p.setOldPosition(current);
                p.setCurrentPosition(new Point2D.Double(newX, newY));
            }

            // Check if turkey collided with the ground.
            if (newY > groundlevel) {
                // Check if turkey landed on right side of mountain. If yes, reset it.
                if (current.getX() > xMountainPeak + (xMountainPeak - leftMountainBase)) {
                    // Randomly choose x-direction for turkey to pace.
                    double paceVelocity;
                    Random r = new Random();
                    double paceDirection = r.nextDouble();
                    // If paceDirection <= 0.5, pace to the left. Otherwise, pace to the right.
                    if (paceDirection <= 0.5) {
                        paceVelocity = -1 * paceSpeed;
                    } else {
                        paceVelocity = paceSpeed;
                    }

                    for (VerletPoint vp : points) {
                        vp.reset(paceVelocity);
                    }
                }
                else {
                    // Apply an equal and opposite impulse, effectively returning turkey to groundlevel and set old y position
                    // to groundlevel to halt y velocity.
                    p.setOldPosition(new Point2D.Double(old.getX(), groundlevel));
                    p.setCurrentPosition(new Point2D.Double(newX, groundlevel));
                    p.addYForce(-1 * p.getYForce());
                    p.addYForce(gravity * timestep);
                }
                isGrounded = true;
            }

            // If turkey hits the left wall, change its pacing velocity to go to the right.
            else if (current.getX() <= 15) {
                paceToRight();
                // Do not consider current x velocity in determining next location since current x velocity is moving to the left.
                // Apply force of turkey "standing up" against gravity.
                p.addYForce(p.getYForce() * -1);
                newX = current.getX() + p.getXForce() * timestep;
                newY = current.getY() + (current.getY() - old.getY() + p.getYForce()) * timestep;
                p.addYForce(gravity * timestep);

                p.setOldPosition(current);
                p.setCurrentPosition(new Point2D.Double(newX, newY));
            }

            // If turkey is on the ground and hits the left base of the mountain and is not already moving to the left, change its pacing
            // velocity to go to the left.
            else if (isGrounded && current.getX() >= leftMountainBase && current.getX() < xMountainPeak && current.getX() - old.getX() >= 0) {
                paceToLeft();
                // Do not consider current x velocity in determining next location since current x velocity is moving to the right.
                // Apply force of turkey standing up against gravity.
                newX = current.getX() + p.getXForce() * timestep;
                p.addYForce(p.getYForce() * -1);
                newY = current.getY() + (current.getY() - old.getY() + p.getYForce()) * timestep;
                p.addYForce(gravity * timestep);

                p.setOldPosition(current);
                p.setCurrentPosition(new Point2D.Double(newX, newY));
            }

            // If turkey landed on left side of the mountain and is not already moving to the left, change its pacing velocity to go to the left.
            else if (current.getX() > leftMountainBase && current.getX() < xMountainPeak) {
                // Check for collision with mountain.

                // Find left endpoint of section of mountain (each "section" is 10 pixels long, corresponding to where coarse noise was added)
                // that current xPosition is in. Each index of points array of mountainShape represents points 5 pixels apart.
                int point1Index = 0;
                int point2Index = 0;

                for (int i = 0; i < 81; i += 2) {
                    if (mountain.xpoints[i] > current.getX()) {
                        point1Index = i - 2;
                        point2Index = i;
                        break;
                    }
                }

                // Determine if trajectory of turkey crossed this section of the mountain. If yes, a collision occurred.
                Line2D.Double mountainEdge = new Line2D.Double(mountain.xpoints[point1Index], mountain.ypoints[point1Index], mountain.xpoints[point2Index], mountain.ypoints[point2Index]);
                Line2D.Double turkeyTrajectory = new Line2D.Double(current, old);
                boolean intersected = turkeyTrajectory.intersectsLine(mountainEdge);
                if (intersected) {
                    // If turkey is not already pacing to the left, change its direction.
                    if (current.getX() - old.getX() >= 0) {
                        paceToLeft();
                    }

                    // Determine height of mountain at x coordinate of this current point.
                    double slope = ((double) mountain.ypoints[point2Index] - mountain.ypoints[point1Index]) / (point2Index - point1Index);
                    int mountainHeightAtX = (int) (slope * (current.getX() - point1Index * 5) + mountain.ypoints[point1Index]);

                    // Set y position of point to this height and stop y velocity.
                    p.setOldPosition(new Point2D.Double(old.getX(), mountainHeightAtX - 1));
                    p.setCurrentPosition(new Point2D.Double(newX, mountainHeightAtX - 1));

                    // Return turkey to y position just before colliding with mountain.
                    newY = old.getY();

                    p.setOldPosition(current);
                    p.setCurrentPosition(new Point2D.Double(newX, newY));

                    p.addYForce(p.getYForce() * -1);
                    p.addYForce(gravity * timestep);

                } else {
                    // Turkey is in air above mountain, so apply acceleration of gravity.
                    newY = current.getY() + (current.getY() - old.getY() + p.getYForce()) * timestep;
                    p.addYForce(gravity * timestep);
                    p.setOldPosition(current);
                    p.setCurrentPosition(new Point2D.Double(newX, newY));
                }
            }

            // Check if turkey landed on the right side of the mountain. If yes, respawn the turkey.
            else if (current.getX() > leftMountainBase && current.getX() < xMountainPeak + (xMountainPeak - leftMountainBase)) {
                // Check for collision with mountain.

                // Find left endpoint of section of mountain (each "section" is 10 pixels long, corresponding to where coarse noise was added)
                // that current xPosition is in. Each index of points array of mountainShape represents points 5 pixels apart.
                int point1Index = 0;
                int point2Index = 0;

                for (int i = 0; i < 81; i += 2) {
                    if (mountain.xpoints[i] > current.getX()) {
                        point1Index = i - 2;
                        point2Index = i;
                        break;
                    }
                }

                // Determine if trajectory of turkey crossed this section of the mountain. If yes, a collision occurred.
                Line2D.Double mountainEdge = new Line2D.Double(mountain.xpoints[point1Index], mountain.ypoints[point1Index], mountain.xpoints[point2Index], mountain.ypoints[point2Index]);
                Line2D.Double turkeyTrajectory = new Line2D.Double(current, old);
                boolean intersected = turkeyTrajectory.intersectsLine(mountainEdge);
                if (intersected) {
                    // Reset turkey.
                    // Randomly choose x-direction for turkey to pace.
                    System.out.println(" Reset turkey");
                    double paceVelocity;
                    Random r = new Random();
                    double paceDirection = r.nextDouble();
                    // If paceDirection <= 0.5, pace to the left. Otherwise, pace to the right.
                    if (paceDirection <= 0.5) {
                        paceVelocity = -1 * paceSpeed;
                    } else {
                        paceVelocity = paceSpeed;
                    }

                    for (VerletPoint vp : points) {
                        vp.reset(paceVelocity);
                    }
                    isGrounded = true;
                }
            }
        }

        for (VerletLink l : links){
            l.resolveConstraints();
        }

        // Update edges.
        edges.clear();
        edges.add(new TurkeyEdge(points.get(0), points.get(1)));
        edges.add(new TurkeyEdge(points.get(1), points.get(2)));
        edges.add(new TurkeyEdge(points.get(2), points.get(3)));
        edges.add(new TurkeyEdge(points.get(3), points.get(4)));
        edges.add(new TurkeyEdge(points.get(4), points.get(5)));
        edges.add(new TurkeyEdge(points.get(5), points.get(6)));
        edges.add(new TurkeyEdge(points.get(6), points.get(7)));
        edges.add(new TurkeyEdge(points.get(7), points.get(8)));
        edges.add(new TurkeyEdge(points.get(8), points.get(9)));
        edges.add(new TurkeyEdge(points.get(9), points.get(10)));
        edges.add(new TurkeyEdge(points.get(10), points.get(11)));
        edges.add(new TurkeyEdge(points.get(9), points.get(14)));
        edges.add(new TurkeyEdge(points.get(14), points.get(13)));
        edges.add(new TurkeyEdge(points.get(13), points.get(12)));
        edges.add(new TurkeyEdge(points.get(14), points.get(15)));
        edges.add(new TurkeyEdge(points.get(15), points.get(16)));
        edges.add(new TurkeyEdge(points.get(16), points.get(17)));
        edges.add(new TurkeyEdge(points.get(17), points.get(18)));
        edges.add(new TurkeyEdge(points.get(18), points.get(19)));
        edges.add(new TurkeyEdge(points.get(19), points.get(0)));
    }

    public ArrayList<VerletPoint> getPoints(){
        return points;
    }

    public ArrayList<TurkeyEdge> getEdges(){
        return edges;
    }

    public void jump(double windVelocity, double gravity, float timestep){
        if (isGrounded){
            isGrounded = false;
            for (VerletPoint p : points){
                p.addYForce(jumpForce);
            }
            updateLocation(windVelocity, gravity, timestep, mountain);
        }
    }

    private void paceToRight(){
        double paceVelocity  = paceSpeed;
        for (VerletPoint p : points){
            p.addXForce(-1*p.getXForce() + paceVelocity);
        }
    }

    private void paceToLeft(){
        double paceVelocity  = -1*paceSpeed;
        for (VerletPoint p : points){
            p.addXForce(-1*p.getXForce() + paceVelocity);
        }
    }
}
