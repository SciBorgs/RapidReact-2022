package frc.robot.util;

public class SciMath
{
  public static final double TAU = 360;

  public static double normalizeAngle(double angle)
  {
    return angle - (Math.floor(angle / TAU) * TAU);
  }

  public static double normalizeInRange(double value, double min, double max) {
    return (value - min) / (max - min);
  }
}