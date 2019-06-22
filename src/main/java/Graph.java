import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.swing.*;
import java.awt.*;

public class Graph {

	public Graph(final String title){
		XYSeries testSeries = new XYSeries("Test", false);
		testSeries.add(0,0);
		testSeries.add(10,10);

		XYSeriesCollection data = new XYSeriesCollection();
		data.addSeries(testSeries); //Series: 0

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

		CustomColorRenderer renderer = new CustomColorRenderer(true,false);
		renderer.setSeriesStroke(0,new BasicStroke(4f));

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
			return Color.getHSBColor(0f,1.0f,1.0f);
		}
	}
}
