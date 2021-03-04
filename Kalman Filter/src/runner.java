import org.ejml.simple.SimpleMatrix;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.JFreeChart;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.category.DefaultCategoryDataset;

import java.util.Random;

public class runner extends ApplicationFrame {
    public static DefaultCategoryDataset datasetVelocity = new DefaultCategoryDataset();
    public static DefaultCategoryDataset
            datasetPosition = new DefaultCategoryDataset(),
            datasetPositionKalman = new DefaultCategoryDataset(),
            datasetPositionexp = new DefaultCategoryDataset(),
            datasetCorrectionX = new DefaultCategoryDataset(),
            datasetCorrectionY = new DefaultCategoryDataset(),
            datasetCorrection = new DefaultCategoryDataset(),
            datasetError = new DefaultCategoryDataset();

    public static void main(String[] args) {
        //Kalman Filter
        // 10 updates per second
        double dt = 1.0 / 10;
        SimpleMatrix f = new SimpleMatrix(new double[][]{
                {1, 0, dt, 0},
                {0, 1, 0, dt},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        }
        );


        SimpleMatrix G = new SimpleMatrix(new double[][]{
                {0.5 * Math.pow(dt, 2)},
                {0.5 * Math.pow(dt, 2)},
                {dt},
                {dt}
        });

        double sigma_a = 9;
        SimpleMatrix Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);

        SimpleMatrix H = SimpleMatrix.identity(f.numRows());

        double positionVar = 0.5;
        double velocityVar = 0.0001;

        SimpleMatrix R = new SimpleMatrix(new double[][]{
                {positionVar, 0, 0, 0},
                {0, positionVar, 0, 0},
                {0, 0, velocityVar, 0},
                {0, 0, 0, velocityVar}
        });

        Kalman kalmanFilter = new Kalman(f, G, R, Q, H);

        kalmanFilter.setInitalPostion(new SimpleMatrix(new double[][]{
                        {0.1},
                        {0.1},
                        {0},
                        {0}
                }),
                new SimpleMatrix(new double[][]{
                        {0},
                        {0},
                        {9},
                        {15}
                }));

        createDataset(kalmanFilter);
        runner chart = new runner(
                "",
                "", datasetVelocity);

        chart.pack();
        RefineryUtilities.centerFrameOnScreen(chart);
        chart.setVisible(true);


        runner chart1 = new runner(
                "",
                "", datasetPosition);

        chart1.pack();
        RefineryUtilities.centerFrameOnScreen(chart1);
        chart1.setVisible(true);

        runner chart2 = new runner(
                "",
                "", datasetPositionexp);

        chart2.pack();
        RefineryUtilities.centerFrameOnScreen(chart2);
        chart2.setVisible(true);
        runner chart3 = new runner(
                "",
                "", datasetPositionKalman);

        chart3.pack();
        RefineryUtilities.centerFrameOnScreen(chart3);
        chart3.setVisible(true);

        runner chart4 = new runner(
                "",
                "", datasetError);

        chart4.pack();
        RefineryUtilities.centerFrameOnScreen(chart4);
        chart4.setVisible(true);
        runner chart5 = new runner(
                "",
                "", datasetCorrection);

        chart5.pack();
        RefineryUtilities.centerFrameOnScreen(chart5);
        chart5.setVisible(true);
    }

    // Customize Chart
    public runner(String applicationTitle, String chartTitle, DefaultCategoryDataset data) {
        super(applicationTitle);
        JFreeChart lineChart = ChartFactory.createLineChart(
                chartTitle,
                "X", "Y",
                data,
                PlotOrientation.VERTICAL,
                true, true, true);

        ChartPanel chartPanel = new ChartPanel(lineChart);
        chartPanel.setPreferredSize(new java.awt.Dimension(560, 367));
        setContentPane(chartPanel);

    }

    private static void createDataset(Kalman kalman) {
        PID pidPositional = new PID(0.7, 0.2, 0.1);
        Random r = new Random();
        kalman.setInitalPostion(new SimpleMatrix(new double[][]{
                        {0},
                        {0},
                        {0},
                        {0}
                }),
                new SimpleMatrix(new double[][]{
                        {0, 0, 0, 0},
                        {0, 0, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 5}
                }));

        SimpleMatrix targetVector = new SimpleMatrix(new double[][]{
                {5},
                {0}
        });
        for (double i = 0; i < 5; i += 0.1) {
            double xP = i + r.nextGaussian() / 5;
            double yP = i * 8.6 / 5.0 + r.nextGaussian() / 5;
            double xV = 1 + r.nextGaussian() / 5;
            double yV = 5 + r.nextGaussian() / 5;
            SimpleMatrix z_k = new SimpleMatrix(new double[][]{
                    {xP},
                    {yP},
                    {xV},
                    {yV}
            });
            z_k = kalman.update(z_k);

            SimpleMatrix currentPositionVector = new SimpleMatrix(new double[][]{
                    {kalman.getXPosition()},
                    {kalman.getYPosition()}
            });

            SimpleMatrix errorVector = targetVector.minus(currentPositionVector);
            double errorMagnitude = Math.sqrt(Math.pow(errorVector.get(0, 0), 2) + Math.pow(errorVector.get(0, 0), 2));
            double correction = pidPositional.correction(errorMagnitude, i+0.1);
            System.out.println(correction);

            double a = xP;
            datasetPositionexp.addValue(yP, "position exp", a + "");
            a = i;
            datasetPosition.addValue(i * 8.6 / 5.0, "position actual", a + "");
            a = z_k.get(0, 0);
            datasetPositionKalman.addValue(z_k.get(1, 0), "position kalman", a + "");

            datasetCorrection.addValue(correction, "correction",i+ "");
            datasetError.addValue(errorMagnitude,"error mag",i+"");

            datasetVelocity.addValue(xV, "x velocity exp", i + "");
            datasetVelocity.addValue(yV, "y velocity exp", i + "");
            datasetVelocity.addValue(z_k.get(2, 0), "x velocity kalman", i + "");
            datasetVelocity.addValue(z_k.get(3, 0), "y velocity kalman", i + "");
            datasetVelocity.addValue(1, "x velocity ideal", i + "");
            datasetVelocity.addValue(5, "y velocity ideal", i + "");
        }

    }

}
