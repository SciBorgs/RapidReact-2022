package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.PortMap;
import frc.robot.sciSensors.SciEncoder;
import frc.robot.sciSensors.SciPigeon;
import frc.robot.sciSensors.SciSpark;
import frc.robot.util.Blockable;
import frc.robot.util.Util;
import frc.robot.Robot;

@Blockable
public class DriveSubsystem extends SubsystemBase {
    private SciEncoder lEncoder, rEncoder;
    public SciPigeon pigeon;
    private BasePigeonSimCollection pigeonSim;

    private final SciSpark[] leftSparks = {
            new SciSpark(PortMap.LEFT_FRONT_SPARK),
            new SciSpark(PortMap.LEFT_MIDDLE_SPARK),
            new SciSpark(PortMap.LEFT_BACK_SPARK)
    };

    private final SciSpark[] rightSparks = {
            new SciSpark(PortMap.RIGHT_FRONT_SPARK),
            new SciSpark(PortMap.RIGHT_MIDDLE_SPARK),
            new SciSpark(PortMap.RIGHT_BACK_SPARK)
    };

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftSparks); 
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightSparks);
    private final Iterable<SciSpark> allSparks = Util.concat(leftSparks, rightSparks);

    private final DifferentialDrive drive = new DifferentialDrive(
            leftGroup,
            rightGroup);

    public DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.ROBOT_WIDTH);

    private PIDController leftFeedback = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private PIDController rightFeedback = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    public enum DriveMode {
        TANK,
        ARCADE,
        CURVATURE
    }

    public DriveSubsystem() {
        lEncoder = new SciEncoder(1, 1, leftSparks);
        rEncoder = new SciEncoder(1, 1, rightSparks);
        resetEncoders();
        
        for (SciSpark motor : allSparks) {
            motor.setIdleMode(IdleMode.kBrake);
            motor.setSmartCurrentLimit(20);
        }

        leftGroup.setInverted(true);
        drive.setDeadband(0.05);

        pigeon = new SciPigeon(PortMap.PIGEON_ID);
        pigeonSim = pigeon.getSimCollection();
        
        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        if(Robot.isReal()) {
            leftGroup.setVoltage(leftVolts);
            rightGroup.setVoltage(rightVolts);
            drive.feed();
        }
        else {
            for (SciSpark s : leftSparks) {
                s.setVoltage(leftVolts);
            } 
            for (SciSpark s : rightSparks) {
                s.setVoltage(rightVolts);
            }  
        }

    }    

    public void setSpeed(DifferentialDriveWheelSpeeds speeds) {
        double leftFeedForward = feedforward.calculate(speeds.leftMetersPerSecond);
        double rightFeedForward = feedforward.calculate(speeds.rightMetersPerSecond);

        double leftOutput = leftFeedback.calculate(lEncoder.getSpeed(), speeds.leftMetersPerSecond);
        double rightOutput = rightFeedback.calculate(rEncoder.getSpeed(), speeds.rightMetersPerSecond);

        leftGroup.setVoltage(leftOutput + leftFeedForward);
        rightGroup.setVoltage(rightOutput + rightFeedForward);
    }

    public void drive(double left, double right) {
        for (SciSpark s : leftSparks) {
            s.setVoltage(left);
        } 
        for (SciSpark s : rightSparks) {
            s.setVoltage(right);
        }  

    }

    public void driveRobot(DriveMode mode, double left, double right) {
        // Controller interface
        switch (mode) {
            case TANK:
                drive.tankDrive(left, right, true);
                break;
            case ARCADE:
                drive.arcadeDrive(left, right);
                break;
            case CURVATURE:
                drive.curvatureDrive(left, right, true);
                break;
        }
    }

    public void failureWatchdog() {
        for (int i = 0; i < leftSparks.length; i++) {
            if (leftSparks[i].updateFailState()) {
                rightSparks[i].forceFailState(true);
            }
        }
        for (int i = 0; i < rightSparks.length; i++) {
            if (rightSparks[i].updateFailState()) {
                leftSparks[i].forceFailState(true);
            }
        }
    }

    public Rotation2d getRotation() {
        return pigeon.getRotation2d();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftAverageVelocity(), getRightAverageVelocity());
    }

    public void updateOdometry() {
        odometry.update(getRotation(), lEncoder.getDistance(), rEncoder.getDistance());
    }

    public void resetEncoders() {
        lEncoder.reset();
        rEncoder.reset();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, getRotation());
    }

    public double getLeftCurrentAmps() {
        return Util.getAverageOfArray(leftSparks, CANSparkMax::getOutputCurrent);
    }

    public double getRightCurrentAmps() {
        return Util.getAverageOfArray(rightSparks, CANSparkMax::getOutputCurrent);
    }

    public double getLeftAverageVelocity() {
        return Util.getAverageOfArray(leftSparks, CANSparkMax::get);
    }

    public double getRightAverageVelocity() {
        return Util.getAverageOfArray(rightSparks, CANSparkMax::get);
    }

    public PIDController getLeftFeedback() {
        return leftFeedback;
    }

    public PIDController getRightFeedback() {
        return rightFeedback;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Iterable<SciSpark> getAllSparks() { 
        return allSparks;
    }

    public boolean isLeftStalling() {
        boolean current = getLeftCurrentAmps() > Constants.CURRENT_THRESHOLD;
        boolean output = Math.abs(getLeftAverageVelocity()) > Constants.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(lEncoder.getSpeed()) < Constants.VELOCITY_THRESHOLD;
        return (current || output) && velocity;
    }

    public boolean isRightStalling() {
        boolean current = getRightCurrentAmps() > Constants.CURRENT_THRESHOLD;
        boolean output = Math.abs(getRightAverageVelocity()) > Constants.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(rEncoder.getSpeed()) < Constants.VELOCITY_THRESHOLD;
        return (current || output) && velocity;
    }

    public boolean isStalling() {
        return isLeftStalling() || isRightStalling();
    }

    @Override
    public void periodic() {
        for(SciSpark s : allSparks) {
            boolean fail = s.updateFailState();
            if(fail) System.out.println("failed");
        }

        updateOdometry();
    } 
}
