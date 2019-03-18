package comp521.assignment2;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyListener;

public class Display {

    private JFrame frame;
    private String title;
    private int width;
    private int height;
    private Canvas canvas;
    private KeyListener listener;

    public Display(String title, int width, int height, KeyListener listener){
        this.title = title;
        this.width = width;
        this.height = height;
        this.listener = listener;

        createDisplay(title);
    }

    private void createDisplay(String title){
        frame = new JFrame(title);
        frame.setSize(width, height);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setResizable(false);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);

        frame.addKeyListener(listener);

        canvas = new Canvas();
        canvas.setPreferredSize(new Dimension(width, height));
        canvas.setMaximumSize(new Dimension(width, height));
        canvas.setMinimumSize(new Dimension(width, height));

        frame.add(canvas);
        frame.pack();
    }

    public Canvas getCanvas(){
        return canvas;
    }
}
