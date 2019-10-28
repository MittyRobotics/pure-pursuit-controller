package com.amhsrobotics.purepursuit;

import javax.swing.*;
import java.awt.*;

public class TestGraph extends JFrame {

    public JPanel panel1 = new JPanel();

    public JPanel panel2 = new JPanel();

    public JPanel panel3 = new JPanel();

    public JPanel panel4 = new JPanel();

    public TestGraph(){
        super();
        setSize(new Dimension(500,500));
        setVisible(true);
        setLayout(null);

        panel1.setSize(new Dimension(10,10));
        panel1.setBackground(Color.BLACK);
        add(panel1);

        panel2.setSize(new Dimension(10,10));
        panel2.setBackground(Color.DARK_GRAY);
        add(panel2);

        panel3.setSize(new Dimension(10,10));
        panel3.setBackground(Color.BLACK);
        add(panel3);

        panel4.setSize(new Dimension(10,10));
        panel4.setBackground(Color.DARK_GRAY);
        add(panel4);
    }
}
