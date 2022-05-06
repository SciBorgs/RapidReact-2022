package frc.robot.subsystems;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.sciSensorsActuators.SciSpark;
import edu.wpi.first.math.controller.RamseteController;
import frc.robot.util.Blockable;
import frc.robot.util.Util;

@Blockable
public class DriveSubsystem extends SubsystemBase {
    private SciEncoder lEncoder, rEncoder;
    public SciPigeon pigeon;

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

    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    private SimpleMotorFeedforward feedForward;

    private PIDController leftPIDController = new PIDController(1, 0, 0);
    private PIDController rightPIDController = new PIDController(1, 0, 0);
    public RamseteController ramseteController;

    public enum DriveMode {
        TANK,
        ARCADE,
        CURVATURE
    }

    public DriveSubsystem() {
        lEncoder = new SciEncoder(1, 1, leftSparks);
        rEncoder = new SciEncoder(1, 1, rightSparks);

        for (SciSpark motor : allSparks) {
            motor.setIdleMode(IdleMode.kBrake);
            motor.setSmartCurrentLimit(20);
        }

        leftGroup.setInverted(true);
        drive.setDeadband(0.05);

        odometry = new DifferentialDriveOdometry(getRotation(), AutoProfile.STARTING_POSE);
        kinematics = new DifferentialDriveKinematics(Constants.ROBOT_WIDTH);
        feedForward = new SimpleMotorFeedforward(1, 3);
        ramseteController = new RamseteController(); // b and zeta
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(rightVolts);
        drive.feed();
    }    

    public void setSpeed(DifferentialDriveWheelSpeeds speeds) {
        double leftFeedForward = feedForward.calculate(speeds.leftMetersPerSecond);
        double rightFeedForward = feedForward.calculate(speeds.rightMetersPerSecond);

        double leftOutput = leftPIDController.calculate(lEncoder.getSpeed(), speeds.leftMetersPerSecond);
        double rightOutput = rightPIDController.calculate(rEncoder.getSpeed(), speeds.rightMetersPerSecond);

        leftGroup.setVoltage(leftOutput + leftFeedForward);
        rightGroup.setVoltage(rightOutput + rightFeedForward);
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
        return new Rotation2d(pigeon.getAngle());
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

    public static <T> double getAverageOfArray(T[] array, java.util.function.ToDoubleFunction<? super T> arg0) {
        return Arrays.stream(array).mapToDouble(arg0).average().orElse(Double.NaN);
    }

    public double getLeftCurrentAmps() {
        return getAverageOfArray(leftSparks, CANSparkMax::getOutputCurrent);
    }

    public double getRightCurrentAmps() {
        return getAverageOfArray(rightSparks, CANSparkMax::getOutputCurrent);
    }

    public double getLeftAverageVelocity() {
        return getAverageOfArray(leftSparks, CANSparkMax::get);
    }

    public double getRightAverageVelocity() {
        return getAverageOfArray(rightSparks, CANSparkMax::get);
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

    public Command getRamseteCommand(String pathName) {
        Trajectory trajectory = PathPlanner.loadPath(pathName, 8, 5);
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                this::getPose,
                ramseteController,
                feedForward,
                kinematics,
                this::getWheelSpeeds,
                leftPIDController,
                rightPIDController,
                this::tankDriveVolts,
                this);

        return ramseteCommand.andThen(() -> tankDriveVolts(0, 0));
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
