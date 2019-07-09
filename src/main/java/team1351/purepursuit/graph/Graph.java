package team1351.purepursuit.graph;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import team1351.purepursuit.PathGenerator;
import team1351.purepursuit.Waypoint;
import team1351.purepursuit.Path;
import team1351.purepursuit.enums.PathType;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;

public class Graph {


	public double[] colorByVelocity;

	public Graph(final String title) {
		XYSeries pathSeries = new XYSeries("path", false);
		XYSeries velocitySeries = new XYSeries("velocity", false);

		Waypoint[] waypoints = new Waypoint[4];
//		waypoints[0] = new Waypoint(new Point2D.Double(0, 0), new Point2D.Double(2, 0));
//		waypoints[1] = new Waypoint(new Point2D.Double(7, 0), new Point2D.Double(11, 0));
//		waypoints[2] = new Waypoint(new Point2D.Double(14, 20), new Point2D.Double(15, 20));
//		waypoints[3] = new Waypoint(new Point2D.Double(20, 20), new Point2D.Double(22, 20));
		waypoints[0] = new Waypoint(new Point2D.Double(0, 0));
		waypoints[1] = new Waypoint(new Point2D.Double(7, 0));
		waypoints[2] = new Waypoint(new Point2D.Double(14, 20));
		waypoints[3] = new Waypoint(new Point2D.Double(20, 20));
		PathGenerator.getInstance().setPathKCurvature(0.8);

		Path path = PathGenerator.getInstance().generate(waypoints, PathType.CUBIC_HERMITE_PATH,2,5,  200);

		colorByVelocity = new double[path.length()];
		for (int i = 0; i < path.length(); i++) {
			pathSeries.add(path.get(i).getX(), path.get(i).getY());
			velocitySeries.add(path.get(i).getPosition(), path.get(i).getVelocity());

			if(path.get(i).getVelocity() < path.getMaxVelocity()){
				colorByVelocity[i] = path.getMaxVelocity()-path.get(i).getVelocity();
			}
			else{
				colorByVelocity[i] = 0;
			}

		}


		XYSeriesCollection positionData = new XYSeriesCollection();
		positionData.addSeries(pathSeries); //Series: 0

		XYSeriesCollection velocityData = new XYSeriesCollection();
		velocityData.addSeries(velocitySeries); //Series: 0

		for(int i = 0; i < waypoints.length; i++){
			XYSeries waypointSeries = new XYSeries("waypoint" + i, false);
			waypointSeries.add(waypoints[i].getWaypoint().getX(),waypoints[i].getWaypoint().getY());
			//waypointSeries.add(waypoints[i].getHandle().getX(),waypoints[i].getHandle().getY());
			positionData.addSeries(waypointSeries);
		}


		JFreeChart positionChart = ChartFactory.createScatterPlot(
				"Position",
				"X",
				"Y",
				positionData,
				PlotOrientation.VERTICAL,
				true,
				true,
				false
		);

		JFreeChart velocityChart = ChartFactory.createScatterPlot(
				"Velocity",
				"Distance",
				"Velocity",
				velocityData,
				PlotOrientation.VERTICAL,
				true,
				true,
				false
		);

		XYPlot positionPlot = positionChart.getXYPlot();
		XYPlot velocityPlot = velocityChart.getXYPlot();
		CustomColorRenderer renderer = new CustomColorRenderer(true, false);
		renderer.setSeriesStroke(0, new BasicStroke(4f));
		CustomColorRenderer renderer1 = new CustomColorRenderer(true, false);
		renderer1.setSeriesStroke(0, new BasicStroke(4f));
		positionPlot.setRenderer(renderer);
		velocityPlot.setRenderer(renderer1);

		ChartPanel positionPanel = new ChartPanel(positionChart);
		positionPanel.setPreferredSize(new java.awt.Dimension(800, 600));

		ChartPanel velocityPanel = new ChartPanel(velocityChart);
		velocityPanel.setPreferredSize(new java.awt.Dimension(800, 600));

		JFrame f = new JFrame();
		f.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
		f.add(positionPanel);
		f.add(velocityPanel, BorderLayout.SOUTH);
		f.pack();
		f.setLocationRelativeTo(null);
		f.setVisible(true);

	}

	private class CustomColorRenderer extends XYLineAndShapeRenderer {

		public CustomColorRenderer(boolean lines, boolean shapes) {
			super(lines, shapes);
		}

		@Override
		public Paint getItemPaint(int row, int col) {
			if(row == 0 || row == 0){
				return Color.getHSBColor(0.25f - (float) (colorByVelocity[col] / 5) / 4, 1f, 0.75f + (float) (colorByVelocity[col] / 5) / 4);
			}
			else{
				return Color.getHSBColor(0.6f, 1.0f, 1.0f);
			}
		}
	}
}
