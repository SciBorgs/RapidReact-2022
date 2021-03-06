package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * A simple vision subsystem to handle filtering our limelight values.
 * Limelight networktable methods are static so they can be used independently.
 * Use an instance of VisionSubsystem for filtering data.
 * Require VisionSubsystem in any command that uses the filtered data from this subsystem.
 */
public class VisionSubsystem extends SubsystemBase {
    private static final PhotonCamera photonVision = new PhotonCamera("photonvision");

    // filters to smooth x and y difference values
    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
    private final LinearFilter yFilter = LinearFilter.singlePoleIIR(VisionConstants.TIMESCALE, VisionConstants.PERIOD);
    // debouncer to smooth the tv value
    private final Debouncer vFilter = new Debouncer(VisionConstants.TIMESCALE, Debouncer.DebounceType.kBoth);

    // last values for filters
    private double xOffset;
    private double yOffset;
    private boolean visible;

    public VisionSubsystem() {
        setCameraParams(getTable(), "pipeline", 0);
        reset();
    }

    // LIMELIGHT
    // returns limelight network table
    public static NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    // gets specific value from the limelight's network table
    public static double getLimelightData(String var) {
        return getLimelightData(var, 0.0);
    }

    public static double getLimelightData(String var, double defaultVal) {
        return getTable().getEntry(var).getDouble(defaultVal);
    }

    // sets a camera paramter
    public static void setCameraParams(NetworkTable table, String param, int setting) {
        table.getEntry(param).setNumber(setting);
    }

    public static void setCameraParams(NetworkTable table, String param, double setting) {
        table.getEntry(param).setValue(setting);
    }

    // PHOTONVISION
    public static PhotonPipelineResult getResult() {
        return photonVision.getLatestResult();
    }

    public static PhotonTrackedTarget getTarget() {
        return getResult().getBestTarget();
    }

    public static PhotonTrackedTarget getSpecificTarget(int index) {
        return getResult().getTargets().get(index);
    }

    public static void setPipeline(int index){
        photonVision.setPipelineIndex(index);
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
