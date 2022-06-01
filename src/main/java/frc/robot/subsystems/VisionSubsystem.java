package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {    
    // filters to smooth x and y difference values
    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
    private final LinearFilter yFilter = LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
    // debouncer to smooth the tv value
    private final Debouncer vFilter = new Debouncer(VisionConstants.TIMESCALE, Debouncer.DebounceType.kBoth);

    // last values for filters
    private double xOffset;
    private double yOffset;
    private boolean visible;

    private ShuffleboardTab mainTab;

    public VisionSubsystem() {
        setCameraParams(getTable(), "pipeline", 0);
        reset();
        this.mainTab = Shuffleboard.getTab("Limelight");
        mainTab.addNumber("Limelight Distance", this::getDistance);
    }

    // returns limelight network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    // gets specific value from the limelight's network table
    public double getLimelightData(String var) {
        return getLimelightData(var, 0.0);
    }

    public double getLimelightData(String var, double defaultVal) {
        return getTable().getEntry(var).getDouble(defaultVal);
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

    public boolean hasTarget() {
        return visible;
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
        xOffset = xFilter.calculate(getLimelightData("tx", xOffset));
        yOffset = yFilter.calculate(getLimelightData("ty", yOffset));
        visible = vFilter.calculate(getLimelightData("tv") == 1);
    }
}
