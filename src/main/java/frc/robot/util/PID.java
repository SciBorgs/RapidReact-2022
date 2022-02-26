package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class PID {
  private double p, i, d;

  private double integrator, prevErr, prevTime, prevDer;

  private Timer timer;

  public PID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;

    prevErr = Double.POSITIVE_INFINITY;

    timer = new Timer();
    timer.start();
  }

  public double getP() { return this.p; }
  public double getI() { return this.i; }
  public double getD() { return this.d; }

  public double getError() { return this.prevErr; }
  public double getIntegral() { return this.integrator; }
  public double getDerivative() { return this.prevDer; }

  public void setP(double p) { this.p = p; }
  public void setI(double i) { this.i = i; }
  public void setD(double d) { this.d = d; }

  public double getOutput(double err) {
    double der = 0;
    if (prevErr != Double.POSITIVE_INFINITY) {
      der = (err - prevErr) / (timer.get() - prevTime);
    }

    integrator += err;
    prevErr  = err;
    prevTime = timer.get();
    prevDer = der;

    return p * err + i * integrator + d * der;
  }

  public double getOutput(double setPoint, double processVar) {
    return this.getOutput(setPoint - processVar);
  }

  public double getOutput() {
    return p * prevErr + i * integrator + d * prevDer;
  }

  public void reset() {
    this.integrator = this.prevErr = this.prevTime = 0;
    this.timer.reset();
  }
}
