package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
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
    
    private CANSparkMax hood, lmotor, rmotor;
    private SciEncoder flywheelEncoder;
    private SciAbsoluteEncoder hoodEncoder;

    private double encoderOffset = 0;

    private final double LOWER_LIMIT = 35.5; // add for real measurement
    private final double UPPER_LIMIT = 9.2;
    private final double SPEED_LIMIT = 0.1;
    public final double HEIGHT_DIFF = 2.08534;
    public final double CAM_MOUNT_ANGLE = 30;

    // hood angle, adjusted by raise and lower hood commands
    public double goToHoodAngle = 15;

    public ShooterSubsystem() {
        shooterPID = new PID(6.0/360.0, 0, 0);
        shooterShufflePID = new ShufflePID("shooter", shooterPID, "big shell");

        hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        rmotor = new CANSparkMax(PortMap.FLYWHEEL_RIGHT_SPARK, MotorType.kBrushless);
        lmotor = new CANSparkMax(PortMap.FLYWHEEL_LEFT_SPARK, MotorType.kBrushless);
        lmotor.follow(rmotor, true);

        rmotor.setIdleMode(IdleMode.kCoast);
        lmotor.setIdleMode(IdleMode.kCoast);

        rmotor.burnFlash();
        lmotor.burnFlash();

        flywheelEncoder = new SciEncoder(Constants.FLYWHEEL_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE, rmotor.getEncoder());
        hoodEncoder = new SciAbsoluteEncoder(PortMap.HOOD_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);
        encoderOffset = getHoodAngle();
        // hoodEncoder.reset();

        // Robot.networkTableSubsystem.bind("shooter", "ty", () -> Robot.limelightSubsystem.getLimelightTableData("ty") + CAM_MOUNT_ANGLE, 0.0);
        // Robot.networkTableSubsystem.bind("shooter", "distance", this::getDistance, 0.0);
        // Robot.networkTableSubsystem.bind("shooter", "hoodangle", this::getHoodAngle, 321.3);
        // Robot.networkTableSubsystem.bind("shooter", "offset", this::getOffset, 321.3);

        // Robot.networkTableSubsystem.bind("shooter", "sethood", this::moveHood, getHoodAngle());
    }
    
    public double getDistance() {
        return HEIGHT_DIFF / Math.tan(Math.toRadians(Robot.limelightSubsystem.getLimelightTableData("ty") + CAM_MOUNT_ANGLE));
    }

    public double getOffset() {
        return encoderOffset;
    }

    public double getHoodAngle() {
        return hoodEncoder.getAngle();
    }

    public void runFlywheel(double speed) {
        System.out.println("SETTING TO " + speed);
        rmotor.set(speed);
    }

    public void stopFlywheel() {
        rmotor.set(0);
    }
    
    public void resetDistanceSpun() {
        flywheelEncoder.setDistance(0);
    }

    public double getDistanceSpun() {
        return flywheelEncoder.getDistance();
    }

    public void setHoodSpeed(double speed) {
        System.out.println("Curr ang: " + getHoodAngle() + " speed " + speed);
        if (speed > 0 && getHoodAngle() > LOWER_LIMIT) {
            // No up
            System.out.println("Top boundary; cannot go up!");
            speed = 0;
        } else if (speed < 0 && getHoodAngle() < UPPER_LIMIT) {
            // No down
            System.out.println("Bottom boundary; cannot go down!");
            speed = 0;
        }
        hood.set(speed);
    }

    public void moveHood(double angle) {
        angle = translateToEncoder(angle);

        double move = shooterPID.getOutput(angle, getHoodAngle());

        System.out.println("ang " + getHoodAngle() + " targ " + angle + " move " + move);

        // signs are reversed because the encoder returns negative values
        if (angle < UPPER_LIMIT || angle > LOWER_LIMIT) {
            move = 0;
            System.out.println("BOUNDARY");
        }
        hood.set(Util.normalize(move, SPEED_LIMIT));
    }

    // shuffleboard

    public void updateGraphs() {
        // distance.setDouble(getDistance());
        // hoodAngle.setDouble(getHoodAngle());
    }

    public void update() {
        shooterShufflePID.update();
        updateGraphs();
    }

    // normal -> crazy encoder
    private double translateToEncoder(double encoderVal) {
        return LOWER_LIMIT - encoderVal;
    }

    // crazy encoder -> normal
    public double translateFromEncoder(double val) {
        return val - LOWER_LIMIT;
    }
}
