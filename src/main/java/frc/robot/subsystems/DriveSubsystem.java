package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    // private SciPigeon pigeon;

    private final SciSpark[] leftSparks = {
            new SciSpark(PortMap.Drivetrain.LEFT_FRONT_SPARK),
            new SciSpark(PortMap.Drivetrain.LEFT_MIDDLE_SPARK),
            new SciSpark(PortMap.Drivetrain.LEFT_BACK_SPARK)
    };

    private final SciSpark[] rightSparks = {
            new SciSpark(PortMap.Drivetrain.RIGHT_FRONT_SPARK),
            new SciSpark(PortMap.Drivetrain.RIGHT_MIDDLE_SPARK),
            new SciSpark(PortMap.Drivetrain.RIGHT_BACK_SPARK)
    };

    private SciPigeon pigeon;

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

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    private SlewRateLimiter filter1 = new SlewRateLimiter(DriveConstants.MAX_JERK); // used for speed in arcade and curvature, left track in tank
    private SlewRateLimiter filter2 = new SlewRateLimiter(DriveConstants.MAX_JERK); // used for right track in tank

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

    private ShuffleboardTab mainTab;

    public DriveSubsystem() {
        lEncoder = leftSparks[0].getEncoder();
        rEncoder = rightSparks[0].getEncoder();
        System.out.println(lEncoder.getCountsPerRevolution() + " | " + rEncoder.getCountsPerRevolution());
        lEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE * DriveConstants.GEAR_RATIO);
        lEncoder.setVelocityConversionFactor(DriveConstants.GEAR_RATIO);
        rEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE * DriveConstants.GEAR_RATIO);
        rEncoder.setVelocityConversionFactor(DriveConstants.GEAR_RATIO);
        
        resetEncoders();

        for (SciSpark motor : allSparks) {
            motor.setIdleMode(IdleMode.kBrake);
            motor.setSmartCurrentLimit(20);
        }

        left.setInverted(true);
        drive.setDeadband(0.05);

        pigeon = new SciPigeon(PortMap.Drivetrain.PIGEON);
        pigeonSim = pigeon.getSimCollection();

        driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),
                7.29,
                7.5,
                60.0,
                Units.inchesToMeters(3),
                0.7112,
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );

        lEncoderSim = new EncoderSim(PortMap.Drivetrain.LEFT_FRONT_SPARK);
        rEncoderSim = new EncoderSim(PortMap.Drivetrain.RIGHT_FRONT_SPARK);

        odometry = new DifferentialDriveOdometry(getRotation());
        this.mainTab = Shuffleboard.getTab("Drivetrain");
        mainTab.addNumber("Heading", this::getHeading);
        mainTab.addNumber("X", this::getX);
        mainTab.addNumber("Y", this::getY);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        if (Robot.isReal()) {
            leftGroup.setVoltage(leftVolts);
            rightGroup.setVoltage(rightVolts);
        } else { // sim workaround
            leftSparks[0].setVoltage(leftVolts);
            rightSparks[0].setVoltage(rightVolts);
        }
        drive.feed();
    }

    /**
     * Drives the robot according to provided DriveMode
     * @param mode
     * @param first Left input in TANK, speed in ARCADE or CURVATURE
     * @param second Right input in TANK, rotation in ARCADE or CURVATURE
     */
    public void driveRobot(DriveMode mode, double first, double second) {
        // Controller interface
        switch (mode) {
            case TANK:
                drive.tankDrive(-first, -second);
                break;
            case ARCADE:
                drive.arcadeDrive(filter1.calculate(first), second);
                break;
            case CURVATURE:
                drive.curvatureDrive(filter1.calculate(first), second, true);
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
        // return Rotation2d.fromDegrees(0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // just for testing
    public double getX() {
        return getPose().getX();
    }

    public double getY() {
        return getPose().getY();
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
    
    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Iterable<SciSpark> getAllSparks() {
        return allSparks;
    }

    // thanks stuy
    public boolean isLeftStalling() {
        boolean current = getLeftCurrentAmps() > DriveConstants.CURRENT_THRESHOLD;
        boolean output = Math.abs(getLeftAverageVelocity()) > DriveConstants.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(lEncoder.getVelocity()) < DriveConstants.VELOCITY_THRESHOLD;
        return (current || output) && velocity;
    }

    public boolean isRightStalling() {
        boolean current = getRightCurrentAmps() > DriveConstants.CURRENT_THRESHOLD;
        boolean output = Math.abs(getRightAverageVelocity()) > DriveConstants.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(rEncoder.getVelocity()) < DriveConstants.VELOCITY_THRESHOLD;
        return (current || output) && velocity;
    }

    public boolean isStalling() {
        return isLeftStalling() || isRightStalling();
    }

    @Override
    public void periodic() {
        updateOdometry();
        for(SciSpark s : getAllSparks()) { s.updateFailState(); }
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
        lEncoderSim = new EncoderSim(PortMap.Drivetrain.LEFT_FRONT_SPARK);
        rEncoderSim = new EncoderSim(PortMap.Drivetrain.RIGHT_FRONT_SPARK);

        odometry = new DifferentialDriveOdometry(getRotation());
        kinematics = new DifferentialDriveKinematics(DriveConstants.ROBOT_WIDTH);

        feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

        driveSim.setPose(odometry.getPoseMeters());
        field2d.setRobotPose(odometry.getPoseMeters());
    }

}
