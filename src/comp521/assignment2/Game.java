package comp521.assignment2;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.Point2D;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Random;

public class Game implements Runnable{

    private Display display;
    private String title;
    private int width;
    private int height;
    private int groundlevel;
    private Thread thread;
    private boolean running;
    private BufferStrategy bs;
    private Graphics g;

    private Polygon mountain;
    private int mountainPeak;
    private int leftMountainBase;
    private int rightMountainBase;
    private Canon canon;
    private ArrayList<CanonBall> activeCanonBalls;
    private double currentWindForce;
    private long lastFlightTime;

    private BufferedImage cloud1Image;
    private BufferedImage cloud2Image;
    private BufferedImage cloud3Image;
    private Cloud cloud1;
    private Cloud cloud2;
    private Cloud cloud3;

    private ArrayList<Turkey> turkeys;
    private ArrayList<CanonBall> cbToRemove;
    private double gravity;
    private float timestep;


    public Game(String title, int width, int height){
        this.title = title;
        this.width = width;
        this.height = height;
        this.groundlevel = 5*(height/6);
        activeCanonBalls = new ArrayList<CanonBall>();
        turkeys = new ArrayList<Turkey>();
        cbToRemove = new ArrayList<>();
    }

    public void run(){
        init();
        long lastTime = System.nanoTime();
        long windTimer = System.nanoTime();
        long turkeyTimer = System.nanoTime();
        gravity = 1000.0; // 1000 pixels/second/second

        while(running){
            long now = System.nanoTime();
            timestep = now - lastTime;
            // Convert timestep from nanoseconds to seconds.
            timestep = timestep / 1000000000;
            lastTime = now;
            tick(timestep);

            if (now - windTimer >= 500000000){
                // Randomly change wind every 0.5 second (500000000 nanoseconds)
                updateWindForce();
                windTimer = now;
            }
             if (now - turkeyTimer >= 800000000 * 1000){
                // Every 8 seconds, select a random turkey to jump.
                 Random r = new Random();
                 int turkeyToJump = r.nextInt(5);
                 turkeys.get(turkeyToJump).jump(currentWindForce, gravity, timestep);
                 turkeyTimer = now;
             }
            render();
        }
        stop();
    }

    public synchronized void start(){
        if(running){
            return;
        }
        running = true;
        thread = new Thread(this);
        thread.start();
    }

    public synchronized void stop(){
        try {
            if(!running){
                return;
            }
            running = false;
            thread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void tick(float timestep){
        // Update locations of cloud, turkeys, canonballs.
        cloud1.updateLocation(currentWindForce, timestep);
        cloud2.updateLocation(currentWindForce, timestep);
        cloud3.updateLocation(currentWindForce, timestep);

        // Update canonballs.
        for (CanonBall cb : activeCanonBalls){
            // Only apply wind acceleration if canon ball is above mountain peak.
            if (cb.getYCenter() <= mountainPeak) {
                cb.updateVelocity(currentWindForce, gravity, timestep);
                cb.updateLocation(timestep);
            }
            else {
                cb.updateVelocity(0, gravity, timestep);
                cb.updateLocation(timestep);
                // Check if canonball has collided with the ground since it is below the mountain peak.
                if (cb.getYCenter() + 7 > groundlevel) {
                    cb.stop(groundlevel);
                    lastFlightTime = cb.getFlightTime();
                }
            }
            if (!cb.isActive()){
                cbToRemove.add(cb);
            }
        }
        // Remove canonballs that collided with turkeys or have "timed out".
        for (CanonBall cb2 : cbToRemove){
            activeCanonBalls.remove(cb2);
        }
        cbToRemove.clear();

        // Update turkeys.
        for (Turkey t : turkeys){
            t.updateLocation(currentWindForce, gravity, timestep, mountain);
        }
    }

    private void render(){
        bs = display.getCanvas().getBufferStrategy();
        if(bs == null){
            display.getCanvas().createBufferStrategy(3);
            return;
        }
        g = bs.getDrawGraphics();
        // Clear screen before drawing.
        g.clearRect(0,0, width, height);

        // Draw background sky.
        g.setColor(new Color(136, 216, 215));
        g.fillRect(0,0,width,height);

        // Draw clouds.
        g.drawImage(cloud1Image, (int) cloud1.getXCoord(), (int) cloud1.getYCoord(), null);
        g.drawImage(cloud2Image, (int) cloud2.getXCoord(), (int) cloud2.getYCoord(), null);
        g.drawImage(cloud3Image, (int) cloud3.getXCoord(), (int) cloud3.getYCoord(), null);

        // Draw active canonballs.
        g.setColor(Color.black);
        for (CanonBall cb : activeCanonBalls) {
            g.fillOval((int) cb.getxCoord() - 7, (int) cb.getyCoord() - 7, 14,14);
        }

        // Draw canon.
        Graphics2D g2d = (Graphics2D)g.create();
        g2d.rotate(Math.toRadians(canon.getAngle()), canon.getXCoord(), canon.getYCoord());
        g2d.setColor(Color.GRAY);
        g2d.fillRect(canon.getXCoord() - 20, canon.getYCoord() - 85, 40, 80);
        // Draw base of cannon (half of it will be concealed by the ground).
        g.setColor(Color.red);
        g.fillOval(canon.getXCoord() - 50,canon.getYCoord()- 50,100,100);

        // Draw flat ground.
        g.setColor(new Color(47, 146, 54));
        g.fillRect(0,groundlevel,width,height/6 + 2);

        // Draw mountain.
        g.fillPolygon(mountain);

        // Draw wall on left side of window.
        g.setColor(Color.darkGray);
        g.fillRect(0, 0, 15, 5*(height/6));

        // Display flight time of last canonball fired. Flight time is 0 if no canonballs have been fired.
        g2d = (Graphics2D)g.create();
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        Font font = new Font("Serif", Font.PLAIN, 20);
        g2d.setFont(font);
        g2d.setColor(Color.blue);
        g2d.drawString("Flight Time: " + lastFlightTime + " ns", 20, 20);

        // Draw turkeys.
        g2d.setColor(Color.black);
        for (Turkey t : turkeys){
            for (VerletPoint vp : t.getPoints()){
                Point2D.Double p = vp.getCurrent();
                g2d.fillOval((int) p.getX() - 2, (int) p.getY() - 2, 4, 4);
            }
            for (TurkeyEdge e : t.getEdges()){
                g2d.draw(e.getLineSegment());
            }
        }

        // Finish drawing.
        bs.show();
        g.dispose();
        g2d.dispose();
    }

    private void init(){
        display = new Display(title, width, height, new KeyListener(){

            @Override
            public void keyTyped(KeyEvent e) {
            }

            @Override
            public void keyPressed(KeyEvent e) {
                // If up arrow or down error is pressed, adjust cannon angle. If SPACE is pressed, fire canon.
                int keyCode = e.getKeyCode();
                switch (keyCode) {
                    case KeyEvent.VK_UP:
                        canon.incrementAngle();
                        break;
                    case KeyEvent.VK_DOWN:
                        canon.decrementAngle();
                        break;
                    case KeyEvent.VK_SPACE:
                        fireCanon(timestep);
                        break;
                }
            }

            @Override
            public void keyReleased(KeyEvent e) {
            }
        });

        // Generate clouds
        cloud1 = new Cloud(200, 25);
        cloud2 = new Cloud(600, 125);
        cloud3 = new Cloud(-400, 80);
        try {
            URL url1 = this.getClass().getResource("/cloud1.png");
            URL url2 = this.getClass().getResource("/cloud2.png");

            cloud1Image = ImageIO.read(url1);
            cloud2Image = ImageIO.read(url2);
            cloud3Image = ImageIO.read(url2);


            //   cloud1Image = ImageIO.read(new File("src\\comp521\\assignment2\\resources\\cloud1.png"));
         //   cloud2Image = ImageIO.read(new File("src\\comp521\\assignment2\\resources\\cloud2.png"));
         //   cloud3Image = ImageIO.read(new File("src\\comp521\\assignment2\\resources\\cloud2.png"));

        } catch (IOException e) {
            System.out.println("Error loading images");
            e.printStackTrace();
        }

        generateMountain();

        canon = new Canon(920, groundlevel - 20, 315);

        updateWindForce();

        // Generate and add 5 turkeys at random locations on the ground to the left of the mountain.
        Random r = new Random();
        double paceSpeed = r.nextDouble()*10 + 20;

        int offset = r.nextInt(width/2 - 240);
        Turkey t1 = new Turkey(offset, paceSpeed, mountain, groundlevel, width/2 - 200, width/2, mountainPeak, gravity);
        offset = r.nextInt(width/2 - 240);
        Turkey t2 = new Turkey(offset, paceSpeed, mountain, groundlevel, width/2 - 200, width/2, mountainPeak, gravity);
        offset = r.nextInt(width/2 - 240);
        Turkey t3 = new Turkey(offset, paceSpeed, mountain, groundlevel, width/2 - 200, width/2, mountainPeak, gravity);
        offset = r.nextInt(width/2 - 240);
        Turkey t4 = new Turkey(offset, paceSpeed, mountain, groundlevel, width/2 - 200, width/2, mountainPeak, gravity);
        offset = r.nextInt(width/2 - 240);
        Turkey t5 = new Turkey(offset, paceSpeed, mountain, groundlevel, width/2 - 200, width/2, mountainPeak, gravity);

        turkeys.add(t1);
        turkeys.add(t2);
        turkeys.add(t3);
        turkeys.add(t4);
        turkeys.add(t5);
    }

    private void generateMountain(){
        mountain = new Polygon();
        mountainPeak = groundlevel - 300;
        leftMountainBase = width/2 - 200;
        rightMountainBase = width/2 + 200;

        // Add points between endpoints. Each point is 5 pixels apart on the x axis.
        int range = rightMountainBase - leftMountainBase;
        for (int i = 0; i <= range/5; i++){
                mountain.addPoint(leftMountainBase + 5*i, groundlevel);
        }

        // Set peak of mountain.
        mountain.ypoints[(range/5)/2] = mountainPeak;

        // Use midpoint bisection to recursively add noise of both coarse and fine scale.
        addCoarseNoise(mountain, 0, (range/5)/2);
        addCoarseNoise(mountain, (range/5)/2, range/5);
    }

    private void addCoarseNoise(Polygon mountain, int start, int end){
        // Base case: start and end are 10 points apart, so begin adding fine noise.
        if (end - start <= 10){
            addFineNoise(mountain, start, end);
            return;
        }
        else {
            // Recursively add random coarse noise (ranges from 0 to 50) to midpoint of indicated section.
            // Determine expected y value of line segment between start and end (y = mx + b), then add coarse noise to the midpoint in either the
            // positive or negative direction.
            int midpoint = (end + start) / 2;
            double slope = ((double) mountain.ypoints[end] - mountain.ypoints[start]) / (end - start);
            int expectedY = (int) (slope * (midpoint - start)) + mountain.ypoints[start];

            Random rand = new Random();
            int coarseNoise = rand.nextInt(50);
            double noiseDirection = rand.nextDouble();

            if (noiseDirection <= 0.5) {
                coarseNoise = coarseNoise * -1;
            }

            int yWithNoise = expectedY + coarseNoise;
            mountain.ypoints[midpoint] = yWithNoise;

            addCoarseNoise(mountain, start, midpoint);
            addCoarseNoise(mountain, midpoint, end);
        }
    }

    private void addFineNoise(Polygon mountain, int start, int end){
        double slope = (mountain.ypoints[end] - mountain.ypoints[start]) / (end - start);
        int expectedY;
        Random rand = new Random();
        int fineNoise;
        double noiseDirection;

        // Base case: start and end are 5 points apart, so add noise individually to each of these points.
        if (end - start <= 5) {
            for (int i = start + 1; i < end; i++){
                expectedY = (int) (slope * (i - start)) + mountain.ypoints[start];
                fineNoise = rand.nextInt(10);
                noiseDirection = rand.nextDouble();
                if (noiseDirection <= 0.5) {
                    fineNoise = fineNoise * -1;
                }
                int yWithNoise = expectedY + fineNoise;
                mountain.ypoints[i] = yWithNoise;
            }
            return;
        }
        else {
            // Recursively add random fine noise (ranges from 0 to 10) to midpoint of indicated section.
            // Determine expected y value of line segment between start and end (y = mx + b), then add fine noise in either the
            // positive or negative y direction.
            int midpoint = (end + start) / 2;
            expectedY = (int) (slope * (midpoint - start)) + mountain.ypoints[start];

            fineNoise = rand.nextInt(10);
            noiseDirection = rand.nextDouble();

            if (noiseDirection <= 0.5) {
                fineNoise = fineNoise * -1;
            }

            int yWithNoise = expectedY + fineNoise;
            mountain.ypoints[midpoint] = yWithNoise;

            addFineNoise(mountain, start, midpoint);
            addFineNoise(mountain, midpoint, end);
        }
    }

    private void fireCanon(float timestep){
        CanonBall cb = new CanonBall(7, canon.getXCoord(), canon.getYCoord(), 1000, canon.getAngle() - 90, mountain, groundlevel, turkeys, leftMountainBase, rightMountainBase);
        activeCanonBalls.add(cb);
        cb.updateVelocity(0, gravity, timestep);
        cb.updateLocation(timestep);
    }

    private void updateWindForce(){
        currentWindForce = (new Random().nextDouble())*200 - 100;
    }
}
