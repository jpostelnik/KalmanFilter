import org.ejml.simple.SimpleMatrix;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.JFreeChart;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.category.DefaultCategoryDataset;


public class Kalman {
    //
    private SimpleMatrix x_k_1, P_k_1;
    //system paramters
    private SimpleMatrix F, G, R, Q, H, I;

    public Kalman(SimpleMatrix f, SimpleMatrix g, SimpleMatrix r, SimpleMatrix q, SimpleMatrix h) {
        F = f;
        G = g;
        R = r;
        Q = q;
        H = h;
        I = SimpleMatrix.identity(F.numRows());
    }

    public void setInitalPostion(SimpleMatrix x_0, SimpleMatrix P_0) {
        x_k_1 = x_0;
        P_k_1 = P_0;
    }


    /**
     * @param z_k - observation
     * @return
     */
    public SimpleMatrix update(SimpleMatrix z_k) {
        //prediction
        SimpleMatrix x_k_est = F.mult(x_k_1);
        SimpleMatrix P_k_est = F.mult(P_k_1).mult(F.transpose()).plus(Q);

        //update
        SimpleMatrix y_k = z_k.minus(H.mult(x_k_est));
        SimpleMatrix S_k = H.mult(P_k_est).mult(H.transpose()).plus(R);
        SimpleMatrix K = P_k_est.mult(H.transpose().mult(S_k.invert()));
        SimpleMatrix x_k = x_k_est.plus(K.mult(y_k));
        SimpleMatrix P_k = (I.minus(K.mult(H))).mult(P_k_est);

        this.x_k_1 = x_k;
        this.P_k_1 = P_k;

        return x_k;
    }

    public double getXPosition() {
        return x_k_1.get(0, 0);
    }

    public double getYPosition() {
        return x_k_1.get(1, 0);
    }

}
