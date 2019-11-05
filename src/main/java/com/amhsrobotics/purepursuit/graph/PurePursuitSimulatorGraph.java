package com.amhsrobotics.purepursuit.graph;

import com.amhsrobotics.purepursuit.PathFollowerPosition;
import com.amhsrobotics.purepursuit.paths.Path;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.chart.title.LegendTitle;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.swing.*;
import java.awt.*;

public class PurePursuitSimulatorGraph extends JFrame {

    private static PurePursuitSimulatorGraph instance = new PurePursuitSimulatorGraph();

    public static PurePursuitSimulatorGraph getInstance(){
        return instance;
    }

    private XYSeriesCollection pathDataset;
    private XYSeriesCollection circleDataset;
    private XYSeriesCollection robotDataset;
    private XYSeriesCollection targetPointDataset;
    private XYSeriesCollection robotVelocityDataset;
    private XYSeriesCollection debugDataset;
    private XYSeriesCollection velocityDataset;


    private XYPlot plot;
    private ChartPanel chart;

    private Path currentPath;

    private PurePursuitSimulatorGraph(){
        super("Graph");

        pathDataset = new XYSeriesCollection();
        circleDataset = new XYSeriesCollection();
        robotDataset = new XYSeriesCollection();
        targetPointDataset = new XYSeriesCollection();
        robotVelocityDataset = new XYSeriesCollection();
        debugDataset = new XYSeriesCollection();
        velocityDataset = new XYSeriesCollection();
        // Create chart
        JFreeChart chart = ChartFactory.createScatterPlot(
                "Path graph",
                "X-Axis", "Y-Axis", null);




        //Changes background color
        XYPlot plot = (XYPlot)chart.getPlot();


        // Create Panel
        ChartPanel panel = new ChartPanel(chart);

        this.chart = panel;

        plot.setDataset(0, debugDataset);
        plot.setDataset(1,targetPointDataset);
        plot.setDataset(2,robotDataset);
        plot.setDataset(3, robotVelocityDataset);
        plot.setDataset(4,circleDataset);





        plot.setDataset(plot.getDatasetCount(),pathDataset);



        plot.setRenderer(plot.getDatasetCount()-1, new PathRenderer(true,true, new Rectangle(1,1)));

        plot.setRenderer(0,  new CustomRenderer(true,true,Color.MAGENTA, new Rectangle(4,4)));


        plot.setRenderer(1,  new CustomRenderer(false,true,Color.GREEN, new Rectangle(4,4)));

        plot.setRenderer(2,  new CustomRenderer(true,true,Color.RED, new Rectangle(1,1)));

        plot.setRenderer(3,  new CustomRenderer(true,true,Color.RED, new Rectangle(1,1)));


        plot.setRenderer(4,  new CustomRenderer(false,true,Color.BLUE, new Rectangle(1,1)));


        this.plot = plot;


        panel.setPreferredSize(new Dimension(600,600));
        
        setLayout(new GridLayout(2,2));
        
        
        add(panel);
        add(createVelocityGraph());

        pack();
        setVisible(true);
    
        chart.setBackgroundPaint(new Color(71, 71, 71));
        chart.getTitle().setPaint(new Color(158, 159, 157));
    
        chart.getLegend().setBackgroundPaint(new Color(71, 71, 71));
        LegendTitle title = chart.getLegend();
        title.setItemPaint(new Color(158, 159, 157));
    
        plot.setBackgroundPaint(Color.DARK_GRAY);
    
        plot.getDomainAxis().setLabelPaint(new Color(158, 159, 157));
        plot.getRangeAxis().setLabelPaint(new Color(158, 159, 157));
        plot.getDomainAxis().setTickLabelPaint(new Color(158, 159, 157));
        plot.getRangeAxis().setTickLabelPaint(new Color(158, 159, 157));
    
        plot.setDomainGridlinePaint(new Color(0,0,0, 180));
        plot.setRangeGridlinePaint(new Color(0,0,0, 180));
        panel.setBackground(new Color(71, 71, 71));
    }

    private ChartPanel createVelocityGraph(){


        velocityDataset.addSeries(new XYSeries("Robot Velocity Over Time"));
        // Create chart
        JFreeChart chart = ChartFactory.createScatterPlot(
                "Velocity graph",
                "Time (s)", "Velocity (in/s)", velocityDataset);



        //Changes background color
        XYPlot plot = (XYPlot)chart.getPlot();

        plot.setRenderer(0, new ColorByVelocityRenderer(true,true, new Rectangle(1,1)));
        // Create Panel
        ChartPanel panel = new ChartPanel(chart);

        panel.setPreferredSize(new Dimension(800,800));
    
        chart.setBackgroundPaint(new Color(71, 71, 71));
        chart.getTitle().setPaint(new Color(158, 159, 157));
    
        chart.getLegend().setBackgroundPaint(new Color(71, 71, 71));
        LegendTitle title = chart.getLegend();
        title.setItemPaint(new Color(158, 159, 157));
    
        plot.setBackgroundPaint(Color.DARK_GRAY);
    
        plot.getDomainAxis().setLabelPaint(new Color(158, 159, 157));
        plot.getRangeAxis().setLabelPaint(new Color(158, 159, 157));
        plot.getDomainAxis().setTickLabelPaint(new Color(158, 159, 157));
        plot.getRangeAxis().setTickLabelPaint(new Color(158, 159, 157));
    
        plot.setDomainGridlinePaint(new Color(0,0,0, 180));
        plot.setRangeGridlinePaint(new Color(0,0,0, 180));
    
        panel.setBackground(new Color(71, 71, 71));
        panel.setPreferredSize(new Dimension(400,400));
        return panel;
    }

    public void graphPathVelocity(Path path){
        XYSeries series = new XYSeries("Path Velocity");
        for(int i = 0; i < path.getTrajectoryPoints().length; i++){
            series.add(path.getTrajectoryPoints()[i].getTime(),path.getTrajectoryPoints()[i].getVelocity());
        }
        velocityDataset.addSeries(series);
    }

    public void graphPath(Path path){
        this.currentPath = path;
        XYSeries series = new XYSeries("Path",false);
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



    public void graphRobotPoint(double x, double y, boolean showTrail){
       XYSeries series;
       if(showTrail){
           if(robotDataset.getSeries().size() == 0){
               series = new XYSeries("Robot",false);

           } else{
               series = robotDataset.getSeries(0);
           }
       }
       else{
           series = new XYSeries("Robot",false);
       }


        series.add(x,y);
        robotDataset.removeAllSeries();
        robotDataset.addSeries(series);
    }

    public void graphTargetPoint(double x, double y){
        XYSeries series = new XYSeries("Target Point");
        series.add(x,y);
        targetPointDataset.removeAllSeries();
        targetPointDataset.addSeries(series);
    }

    public void graphVelocity(double leftVelocity, double rightVelocity, double robotWidth){
        XYSeries lSeries = new XYSeries("Left Velocity");
        XYSeries rSeries = new XYSeries("Right Velocity");
        double heading = PathFollowerPosition.getInstance().getPathCentricHeading();
        double x1 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading-90))*(robotWidth/2);
        double y1 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading-90))*(robotWidth/2);
        double x2 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading+90))*(robotWidth/2);
        double y2 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading+90))*(robotWidth/2);
        double x3 = x1 + Math.cos(Math.toRadians(heading))*(rightVelocity/.1);
        double y3 = y1 + Math.sin(Math.toRadians(heading))*(rightVelocity/.1);
        double x4 = x2 + Math.cos(Math.toRadians(heading))*(leftVelocity/.1);
        double y4 = y2 + Math.sin(Math.toRadians(heading))*(leftVelocity/.1);
        rSeries.add(x1,y1);
        lSeries.add(x2,y2);
        rSeries.add(x3,y3);
        lSeries.add(x4,y4);
        robotVelocityDataset.removeAllSeries();
        robotVelocityDataset.addSeries(lSeries);
        robotVelocityDataset.addSeries(rSeries);
    }

    public void graphRobotVelocityOverTime(double avgVelocity, double t){
        velocityDataset.getSeries(0).add(t,avgVelocity);
    }

    public void graphDebug(double x, double y, double x1, double y1, double x2, double y2){
        XYSeries series = new XYSeries("debug1");
        XYSeries series1 = new XYSeries("debug2");

        series.add(x,y);
        series.add(x1,y1);
        series1.add(x2,y2);

        debugDataset.removeAllSeries();

        debugDataset.addSeries(series);
        debugDataset.addSeries(series1);

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
        domain.setRange(lowerRange-20, upperRange+20);
        domain.setVerticalTickLabels(true);
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        range.setRange(lowerRange-20, upperRange+20);

    }



    public XYSeriesCollection getPathDataset() {
        return pathDataset;
    }

    public void setPathDataset(XYSeriesCollection pathDataset) {
        this.pathDataset = pathDataset;
    }

    private class CustomRenderer extends XYLineAndShapeRenderer {

        private Color color;
        private Shape shape;

        public CustomRenderer(boolean lines, boolean shapes, Color itemColor, Shape shape) {
            super(lines, shapes);
            this.color = itemColor;
            this.shape = shape;
        }

        @Override
        public Shape getItemShape(int row, int column) {
            return shape;
        }

        @Override
        public Paint getItemPaint(int row, int col) {
            return color;
        }
    }
    private class ColorByVelocityRenderer extends XYLineAndShapeRenderer {

        private Shape shape;

        public ColorByVelocityRenderer(boolean lines, boolean shapes, Shape shape) {
            super(lines, shapes);
            this.shape = shape;
        }

        @Override
        public Shape getItemShape(int row, int column) {
            return shape;
        }

        @Override
        public Paint getItemPaint(int row, int col) {
            if(row == 1){
                double velocity = currentPath.getTrajectoryPoints()[col].getVelocity();
                double upperBounds = 0;
                for (int i = 0; i < currentPath.getTrajectoryPoints().length; i++) {
                    if (currentPath.getTrajectoryPoints()[i].getVelocity() > upperBounds) {
                        upperBounds = currentPath.getTrajectoryPoints()[i].getVelocity();
                    }
                }

                return Color.getHSBColor((float) (velocity / upperBounds) / 4, 1f, 0.75f);


            }
            else{
                return Color.WHITE;
            }

        }

    }
    private class PathRenderer extends XYLineAndShapeRenderer {

        private Shape shape;

        public PathRenderer(boolean lines, boolean shapes, Shape shape) {
            super(lines, shapes);
            this.shape = shape;
        }

        @Override
        public Shape getItemShape(int row, int column) {
            return shape;
        }

        @Override
        public Paint getItemPaint(int row, int col) {
            if (row == 0) {
                double velocity = currentPath.getTrajectoryPoints()[col].getVelocity();
                double upperBounds = 0;
                for (int i = 0; i < currentPath.getTrajectoryPoints().length; i++) {
                    if (currentPath.getTrajectoryPoints()[i].getVelocity() > upperBounds) {
                        upperBounds = currentPath.getTrajectoryPoints()[i].getVelocity();
                    }
                }

                return Color.getHSBColor((float) (velocity / upperBounds) / 4, 1f, 0.75f);


            } else {
                return Color.BLACK;
            }

        }
    }

}
