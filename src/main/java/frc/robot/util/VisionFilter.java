package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

/** A class to filter 3 limelight values at once: instantiate and call update() once per tick */
public class VisionFilter {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  static {
    setCameraParams("pipeline", 0);
  }

  // filters to smooth x and y difference values
  private final LinearFilter xFilter =
      LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
  private final LinearFilter yFilter =
      LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
  // debouncer to smooth the tv value
  private final Debouncer vFilter =
      new Debouncer(VisionConstants.TIMESCALE, Debouncer.DebounceType.kBoth);

  // last values for filters
  private double xOffset;
  private double yOffset;
  private boolean visible;

  // gets specific value from the limelight's network table
  public static double getLimelightData(String var) {
    return getLimelightData(var, 0.0);
  }

  public static double getLimelightData(String var, double defaultVal) {
    return table.getEntry(var).getDouble(defaultVal);
  }

  // sets a camera paramter
  public static void setCameraParams(String param, int setting) {
    table.getEntry(param).setNumber(setting);
  }

  public static void setCameraParams(String param, double setting) {
    table.getEntry(param).setValue(setting);
  }

  // gets the distance from the limelight to the target
  public double getDistance() {
    return VisionConstants.HEIGHT_DIFF
        / Math.tan(Math.toRadians(yOffset + VisionConstants.MOUNT_ANGLE));
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public boolean hasTarget() {
    return visible;
  }

  public void update() {
    xOffset = xFilter.calculate(getLimelightData("tx", xOffset));
    yOffset = yFilter.calculate(getLimelightData("ty", yOffset));
    visible = vFilter.calculate(getLimelightData("tv") == 1);
  }
}
