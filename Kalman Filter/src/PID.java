public class PID {
    //    private  runtime;
    private double kp, ki, kd;
    double[] errorList;
    int index;

    private double totalError​, l​astError​, l​astTime;

    public PID(double kp, double ki, double kd) {
//        this.runtime = runtime;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        l​astTime = 0;
        index = 0;
        errorList = new double[10];
    }

    public void reset() {
        totalError​ = l​astError​ = 0;
//        l​astTime = runtime.time() ;
    }

    public double getKp(double error) {
        return kp * error;
    }

    public double getKi(double error) {
        errorList[index] = error;
        totalError​ = 0;
        for (int i = 0; i < errorList.length; i++) {
            totalError​ += errorList[i];
        }
        index = (index + 1) % errorList.length;
        return ki * totalError​;
    }

    public double getKd(double error, double time) {
        return kd * (error - l​astError​) / (time - l​astTime);
    }

    public double correction(double error, double time) {
        totalError​ += error;

        double output = getKp(error) + getKi(error) + getKd(error, time);
        l​astError​ = error;
        l​astTime = time;

        return output;
    }

}
