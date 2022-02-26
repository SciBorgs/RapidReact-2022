package frc.robot.util;

// mainly used for changing pid constants via shuffleboard
// this can be a shared, mutable class between different pids
public class PIDCoeffs {
    private double p, i, d;
    public PIDCoeffs(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
  
    public double getP() { return this.p; }
    public double getI() { return this.i; }
    public double getD() { return this.d; }
  
    public void setP(double p) { this.p = p; }
    public void setI(double i) { this.i = i; }
    public void setD(double d) { this.d = d; }
}