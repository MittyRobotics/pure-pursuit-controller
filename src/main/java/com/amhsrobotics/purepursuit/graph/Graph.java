package com.amhsrobotics.purepursuit.graph;

import com.amhsrobotics.purepursuit.paths.Path;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
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
    private XYSeriesCollection circleDataset;

    private XYPlot plot;
    private ChartPanel chart;

    private Graph(){
        super("Graph");

        pathDataset = new XYSeriesCollection();
        circleDataset = new XYSeriesCollection();
        // Create chart
        JFreeChart chart = ChartFactory.createScatterPlot(
                "Path graph",
                "X-Axis", "Y-Axis", pathDataset);




        //Changes background color
        XYPlot plot = (XYPlot)chart.getPlot();
        plot.setBackgroundPaint(new Color(255,255,255));

        // Create Panel
        ChartPanel panel = new ChartPanel(chart);

        this.chart = panel;

        plot.setDataset(1,circleDataset);
        this.plot = plot;

        panel.setMinimumDrawWidth(0);
        panel.setMaximumDrawWidth(Integer.MAX_VALUE);
        panel.setMinimumDrawHeight(0);
        panel.setMaximumDrawHeight(Integer.MAX_VALUE);

        panel.setPreferredSize(new Dimension(800,800));

        setContentPane(panel);
    }


    public void graphPath(Path path){
        XYSeries series = new XYSeries("Path");
        for(int i = 0; i < path.getTrajectoryPoints().length; i++){
            series.add(path.getTrajectoryPoints()[i].getX(),path.getTrajectoryPoints()[i].getY());
        }

        pathDataset.removeAllSeries();
        pathDataset.addSeries(series);
    }

    public void graphCircle(double x, double y, double radius){
        XYSeries series = new XYSeries("Circle");
        for(int i = 0; i < 360; i++){
                    series.add(x + (Math.cos(Math.toRadians(i)) * radius), y + (Math.sin(Math.toRadians(i) )* radius));
        }
        series.add(x , y);
        circleDataset.removeAllSeries();
        circleDataset.addSeries(series);
    }

    public void resizeGraph(){
        double lowerBound = 9999;
        double upperBound = -9999;
        double leftBound = 9999;
        double rightBound = -9999;
        for(int i = 0; i < plot.getDatasetCount(); i++){
            for(int a = 0; a < plot.getDataset(i).getSeriesCount(); a++){
                for(int j = 0; j < plot.getDataset(i).getItemCount(a); j++){
                    if(plot.getDataset(i).getYValue(a,j) < lowerBound){
                        lowerBound = plot.getDataset(i).getYValue(a,j);
                    }
                    if(plot.getDataset(i).getYValue(a,j) > upperBound){
                        upperBound = plot.getDataset(i).getYValue(a,j);
                    }
                    if(plot.getDataset(i).getXValue(a,j) < leftBound){
                        leftBound = plot.getDataset(i).getXValue(a,j);
                    }
                    if(plot.getDataset(i).getXValue(a,j) > rightBound){
                        rightBound = plot.getDataset(i).getXValue(a,j);
                    }
                }
            }
        }

        double lowerRange=0;
        double upperRange=0;

        if(lowerBound < leftBound){
            lowerRange = lowerBound;
        }
        else{
            lowerRange = leftBound;
        }

        if(upperBound > rightBound){
            upperRange = upperBound;
        }
        else{
            upperRange = rightBound;
        }

        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        domain.setRange(lowerRange-10, upperRange+10);
        domain.setVerticalTickLabels(true);
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        range.setRange(lowerRange-10, upperRange+10);

        System.out.println(lowerBound + " " + upperBound);
    }



    public XYSeriesCollection getPathDataset() {
        return pathDataset;
    }

    public void setPathDataset(XYSeriesCollection pathDataset) {
        this.pathDataset = pathDataset;
    }
}
