import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import pure_pursuit.BezierPoint;
import pure_pursuit.Path;
import pure_pursuit.paths.BezierCurvePath;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;

public class Graph {

	public Graph(final String title) {
		XYSeries pathSeries = new XYSeries("Path", false);
		BezierPoint[] waypoints = new BezierPoint[2];
		waypoints[0] = new BezierPoint(new Point2D.Double(0, 0), new Point2D.Double(10, 0));
		waypoints[1] = new BezierPoint(new Point2D.Double(10, 5), new Point2D.Double(0, 5));

		Path path = new Path(2, 5, new BezierCurvePath(waypoints, 200));
		path.generatePath();
		for (int i = 0; i < path.length(); i++) {
			pathSeries.add(path.getPoint(i).getX(), path.getPoint(i).getY());
		}


		XYSeriesCollection data = new XYSeriesCollection();
		data.addSeries(pathSeries); //Series: 0


		for(int i = 0; i < waypoints.length; i++){
			XYSeries waypointSeries = new XYSeries("waypoint" + i, false);
			waypointSeries.add(waypoints[i].getWaypoint().getX(),waypoints[i].getWaypoint().getY());
			waypointSeries.add(waypoints[i].getHandle().getX(),waypoints[i].getHandle().getY());
			data.addSeries(waypointSeries);
		}


		JFreeChart chart = ChartFactory.createScatterPlot(
				"Graph",
				"X",
				"Y",
				data,
				PlotOrientation.VERTICAL,
				true,
				true,
				false
		);

		XYPlot plot = chart.getXYPlot();

		CustomColorRenderer renderer = new CustomColorRenderer(true, false);
		renderer.setSeriesStroke(0, new BasicStroke(4f));

		plot.setRenderer(renderer);

		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(800, 800));

		JFrame f = new JFrame();
		f.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
		f.add(chartPanel);
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
			return Color.getHSBColor(0.6f, 1.0f, 1.0f);
		}
	}
}
