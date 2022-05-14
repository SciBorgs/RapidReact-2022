package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {    
    // filters to smooth x and y difference values
    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
    private final LinearFilter yFilter = LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);

    // last values for offset
    private double xOffset;
    private double yOffset;

    public VisionSubsystem() {
        reset();
    }

    // returns limelight network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    // gets specific value from the limelight's network table
    public double getLimelightData(String var) {
        return getTable().getEntry(var).getDouble(0.0);
    }

    // sets a camera paramter
    public void setCameraParams(NetworkTable table, String param, int setting) {
        table.getEntry(param).setNumber(setting);
    }

    public void setCameraParams(NetworkTable table, String param, double setting) {
        table.getEntry(param).setValue(setting);
    }

    public double getXOffset() {
        return xOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    // gets the distance from the limelight to the target
    public double getDistance() {
        return VisionConstants.HEIGHT_DIFF / Math.tan(Math.toRadians(yOffset + VisionConstants.MOUNT_ANGLE));
    }

    // this should be called every time the subsystem is scheduled (after not being scheduled)
    public void reset() {
        xFilter.reset();
        yFilter.reset();
        xOffset = 0.0;
        yOffset = 0.0;
    }

    @Override
    public void periodic() {
        xOffset = xFilter.calculate(getLimelightData("tx"));
        yOffset = yFilter.calculate(getLimelightData("ty"));
    }
}
