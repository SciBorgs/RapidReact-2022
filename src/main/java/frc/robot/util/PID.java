package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class PID {
  private double p;
  private double i;
  private double d;

  private double integrator;
  private double prevErr;
  private double prevTime;

  private Timer timer;

  public PID(double p, double i, double d)
  {
    this.p = p;
    this.i = i;
    this.d = d;

    prevErr = Double.POSITIVE_INFINITY;

    timer = new Timer();
    timer.start();
  }

  public double getOutput(double setPoint, double processVar)
  {
    double err = setPoint - processVar;

    double der = 0;
    if (prevErr != Double.POSITIVE_INFINITY) {
      der = (err - prevErr) / (timer.get() - prevTime);
    }

    integrator += err;
    prevErr  = err;
    prevTime = timer.get();

    return p * err + i * integrator + d * der;
  }
}
