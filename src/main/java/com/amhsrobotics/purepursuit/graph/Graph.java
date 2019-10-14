package com.amhsrobotics.purepursuit.graph;

import com.amhsrobotics.purepursuit.paths.Path;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.swing.*;
import java.awt.*;

public class Graph extends JFrame {

    private static Graph instance = new Graph();

    public static Graph getInstance(){
        return instance;
    }

    private XYSeriesCollection pathDataset;

    private Graph(){
        super("Grapg");

        pathDataset = new XYSeriesCollection();

        // Create chart
        JFreeChart chart = ChartFactory.createXYLineChart(
                "Path graph",
                "X-Axis", "Y-Axis", pathDataset);



        //Changes background color
        XYPlot plot = (XYPlot)chart.getPlot();
        plot.setBackgroundPaint(new Color(255,255,255));

        // Create Panel
        ChartPanel panel = new ChartPanel(chart);

        panel.setMinimumDrawWidth(0);
        panel.setMaximumDrawWidth(Integer.MAX_VALUE);
        panel.setMinimumDrawHeight(0);
        panel.setMaximumDrawHeight(Integer.MAX_VALUE);
        setContentPane(panel);
    }

    public void graphPath(Path path){
        XYSeries series = new XYSeries("pathDataset");
        for(int i = 0; i < path.getTrajectoryPoints().length; i++){
            series.add(path.getTrajectoryPoints()[i].getX(),path.getTrajectoryPoints()[i].getY());
        }

        pathDataset.removeAllSeries();
        pathDataset.addSeries(series);
    }


    public XYSeriesCollection getPathDataset() {
        return pathDataset;
    }

    public void setPathDataset(XYSeriesCollection pathDataset) {
        this.pathDataset = pathDataset;
    }
}
