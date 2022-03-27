package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.util.PID;
import frc.robot.util.ShufflePID;
import frc.robot.util.Util;

public class ShooterSubsystem extends SubsystemBase {
    private PID shooterPID;
    private ShufflePID shooterShufflePID;

    private ShuffleboardTab shooterTab;
    private NetworkTableEntry distance, hoodAngle, changeHoodAngle;
    
    private CANSparkMax hood, lmotor, rmotor;
    private SciEncoder flywheelEncoder;
    private SciAbsoluteEncoder hoodEncoder;

    private final double LIMIT = 30; // add for real measurement
    private final double SPEED_LIMIT = 0.1;
    public final double HEIGHT_DIFF = 2.08534;
    public final double CAM_MOUNT_ANGLE = 30;

    public ShooterSubsystem() {
        shooterPID = new PID(0.0005, 0, 0);
        shooterShufflePID = new ShufflePID("shooter", shooterPID, "big shell");

        shooterTab = Shuffleboard.getTab("shooter");
        distance = shooterTab.add("distance", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        hoodAngle = shooterTab.add("hoodangle ", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        changeHoodAngle = shooterTab.add("change hood angle", 0.0).getEntry();

        hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        rmotor = new CANSparkMax(PortMap.FLYWHEEL_RIGHT_SPARK, MotorType.kBrushless);
        lmotor = new CANSparkMax(PortMap.FLYWHEEL_LEFT_SPARK, MotorType.kBrushless);
        lmotor.follow(rmotor);

        flywheelEncoder = new SciEncoder(Constants.FLYWHEEL_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE, rmotor.getEncoder());
        hoodEncoder = new SciAbsoluteEncoder(PortMap.HOOD_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);
    }
    
    public double getDistance() {
        return HEIGHT_DIFF / Math.tan(Math.toRadians(Robot.limelightSubsystem.getLimelightTableData("ty") + CAM_MOUNT_ANGLE));
    }

    public double getCurrentHoodAngle() {
        return hoodEncoder.getAngle();
    }

    public double getRequiredHoodAngle() {
        // return tba(getDistance());
        return 0;
    }

    public void run(double speed) {
        rmotor.set(speed);
    }

    public void stop() {
        rmotor.set(0);
    }

    public void resetDistanceSpun() {
        flywheelEncoder.setDistance(0);
    }

    public double getDistanceSpun() {
        return flywheelEncoder.getDistance();
    }

    public void moveHood(double angle) {
        double turn = -shooterPID.getOutput(angle, hoodEncoder.getAngle());
        hood.set(Util.normalize(turn, SPEED_LIMIT));
    }

    public void updateAngle() { 
        moveHood(changeHoodAngle.getDouble(0.0));
    }

    public void updateGraphs() {
        distance.setDouble(getDistance());
        hoodAngle.setDouble(getCurrentHoodAngle());
    }

    public void update() {
        shooterShufflePID.update();
        updateGraphs();
        updateAngle();
    }
}
