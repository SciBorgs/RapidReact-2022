package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensors.SciPigeon;
import frc.robot.sciSensors.SciSpark;
import frc.robot.util.Blockable;
import frc.robot.util.EncoderSim;
import frc.robot.util.Util;

@Blockable
public class DriveSubsystem extends SubsystemBase {

    private SciPigeon pigeon;

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

    private final RelativeEncoder lEncoder;
    private final RelativeEncoder rEncoder;

    public DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.ROBOT_WIDTH);

    private PIDController leftFeedback = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private PIDController rightFeedback = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV,
            DriveConstants.kA);

    // SIMULATION
    private DifferentialDrivetrainSim driveSim;
    private BasePigeonSimCollection pigeonSim;
    private EncoderSim lEncoderSim, rEncoderSim;
    public Field2d field2d = new Field2d();

    public enum DriveMode {
        TANK,
        ARCADE,
        CURVATURE
    }

    public DriveSubsystem() {
        lEncoder = leftSparks[0].getEncoder();
        rEncoder = rightSparks[0].getEncoder();
        System.out.println(lEncoder.getCountsPerRevolution() + " | " + rEncoder.getCountsPerRevolution());
        lEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE);
        // lEncoder.setVelocityConversionFactor(DriveConstants.) TODO set correct value
        // for this and rEncoder
        rEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE);
        resetEncoders();

        for (SciSpark motor : allSparks) {
            motor.setIdleMode(IdleMode.kBrake);
            motor.setSmartCurrentLimit(20);
        }

        leftGroup.setInverted(true);
        drive.setDeadband(0.05);

        pigeon = new SciPigeon(PortMap.PIGEON_ID);
        pigeonSim = pigeon.getSimCollection();

        // Will change once we get more information on our drivetrain
        driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),
                7.29,
                7.5,
                60.0,
                Units.inchesToMeters(3),
                0.7112,

                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

        lEncoderSim = new EncoderSim(PortMap.LEFT_FRONT_SPARK);
        rEncoderSim = new EncoderSim(PortMap.RIGHT_FRONT_SPARK);

        odometry = new DifferentialDriveOdometry(getRotation());
    }

    public void setSideVoltage(double voltage, SciSpark[] sparks) {
        for (SciSpark s : sparks)
            s.setVoltage(voltage);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        if (Robot.isReal()) {
            leftGroup.setVoltage(leftVolts);
            rightGroup.setVoltage(rightVolts);
        } else { // lmao
            leftSparks[0].setVoltage(leftVolts);
            rightSparks[0].setVoltage(rightVolts);
        }
        drive.feed();
    }

    public void setSpeed(DifferentialDriveWheelSpeeds speeds) {
        double leftFeedForward = feedforward.calculate(speeds.leftMetersPerSecond);
        double rightFeedForward = feedforward.calculate(speeds.rightMetersPerSecond);

        double leftOutput = leftFeedback.calculate(lEncoder.getVelocity(), speeds.leftMetersPerSecond);
        double rightOutput = rightFeedback.calculate(rEncoder.getVelocity(), speeds.rightMetersPerSecond);

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
        return pigeon.getRotation2d();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return getRotation().getDegrees();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftAverageVelocity(), getRightAverageVelocity());
    }

    public void updateOdometry() {
        odometry.update(getRotation(), lEncoder.getPosition(), rEncoder.getPosition());
    }

    public void resetEncoders() {
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        // pigeon.setAngle(pose.getRotation().getDegrees());
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
        boolean velocity = Math.abs(lEncoder.getVelocity()) < Constants.VELOCITY_THRESHOLD;
        return (current || output) && velocity;
    }

    public boolean isRightStalling() {
        boolean current = getRightCurrentAmps() > Constants.CURRENT_THRESHOLD;
        boolean output = Math.abs(getRightAverageVelocity()) > Constants.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(rEncoder.getVelocity()) < Constants.VELOCITY_THRESHOLD;
        return (current || output) && velocity;
    }

    public boolean isStalling() {
        return isLeftStalling() || isRightStalling();
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInputs(leftSparks[0].getAppliedOutput(), rightSparks[0].getAppliedOutput());

        lEncoderSim.setPosition(driveSim.getLeftPositionMeters());
        lEncoderSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond());
        rEncoderSim.setPosition(driveSim.getRightPositionMeters());
        rEncoderSim.setVelocity(driveSim.getRightVelocityMetersPerSecond());
        pigeonSim.setRawHeading(driveSim.getHeading().getDegrees());

        driveSim.update(0.02);
        field2d.setRobotPose(odometry.getPoseMeters());
    }

    public void putTrajectory(Trajectory t, String name) {
        field2d.getObject("Trajectory").setTrajectory(t);
    }

    // reset everything
    public void reset() {
        lEncoderSim = new EncoderSim(PortMap.LEFT_FRONT_SPARK);
        rEncoderSim = new EncoderSim(PortMap.RIGHT_FRONT_SPARK);

        odometry = new DifferentialDriveOdometry(getRotation());
        kinematics = new DifferentialDriveKinematics(DriveConstants.ROBOT_WIDTH);

        leftFeedback = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        rightFeedback = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV,
                DriveConstants.kA);

        driveSim.setPose(odometry.getPoseMeters());
        field2d.setRobotPose(odometry.getPoseMeters());
    }

}
