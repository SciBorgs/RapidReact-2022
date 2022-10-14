package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.util.EncoderSim;
import frc.robot.util.TrajectoryRegister;
import frc.robot.util.Util;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax[] leftSparks = {
    new CANSparkMax(PortMap.Drivetrain.LEFT_FRONT_SPARK, MotorType.kBrushless),
    new CANSparkMax(PortMap.Drivetrain.LEFT_MIDDLE_SPARK, MotorType.kBrushless),
    new CANSparkMax(PortMap.Drivetrain.LEFT_BACK_SPARK, MotorType.kBrushless)
  };

  private final CANSparkMax[] rightSparks = {
    new CANSparkMax(PortMap.Drivetrain.RIGHT_FRONT_SPARK, MotorType.kBrushless),
    new CANSparkMax(PortMap.Drivetrain.RIGHT_MIDDLE_SPARK, MotorType.kBrushless),
    new CANSparkMax(PortMap.Drivetrain.RIGHT_BACK_SPARK, MotorType.kBrushless)
  };

  private WPI_PigeonIMU pigeon = new WPI_PigeonIMU(PortMap.Drivetrain.PIGEON);

  private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftSparks);
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightSparks);
  private final Iterable<CANSparkMax> allSparks = Util.concat(leftSparks, rightSparks);

  // private double speedLimit = 0.7;
  // private SimpleWidget speedLimitWidget;

  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  private final RelativeEncoder lEncoder = leftSparks[0].getEncoder();
  private final RelativeEncoder rEncoder = rightSparks[0].getEncoder();

  public DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.ROBOT_WIDTH);

  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  private SlewRateLimiter filter1 =
      new SlewRateLimiter(DriveConstants.DELTA); // used for speed in arcade and
  // curvature, left track in tank
  private SlewRateLimiter filter2 =
      new SlewRateLimiter(DriveConstants.DELTA); // used for right track in tank

  // SIMULATION
  private DifferentialDrivetrainSim driveSim =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          7.29,
          7.5,
          60.0,
          Units.inchesToMeters(3),
          0.7112,
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  private BasePigeonSimCollection pigeonSim = pigeon.getSimCollection();
  private EncoderSim lEncoderSim = new EncoderSim(PortMap.Drivetrain.LEFT_FRONT_SPARK);
  private EncoderSim rEncoderSim = new EncoderSim(PortMap.Drivetrain.RIGHT_FRONT_SPARK);
  public Field2d field2d = new Field2d();

  public enum DriveMode {
    TANK,
    ARCADE,
    CURVATURE
  }

  private ShuffleboardTab tab;

  public DriveSubsystem() {
    lEncoder.setPositionConversionFactor(
        DriveConstants.WHEEL_CIRCUMFERENCE * DriveConstants.GEAR_RATIO);
    lEncoder.setVelocityConversionFactor(DriveConstants.GEAR_RATIO);
    rEncoder.setPositionConversionFactor(
        DriveConstants.WHEEL_CIRCUMFERENCE * DriveConstants.GEAR_RATIO);
    rEncoder.setVelocityConversionFactor(DriveConstants.GEAR_RATIO);

    resetEncoders();

    for (CANSparkMax motor : allSparks) {
      motor.setIdleMode(IdleMode.kBrake);
      motor.setSmartCurrentLimit(20);
    }

    leftGroup.setInverted(true);
    drive.setDeadband(0.05);

    for (CANSparkMax motor : allSparks) {
      motor.burnFlash();
    }

    odometry = new DifferentialDriveOdometry(getRotation(), new Pose2d(10, 4, getRotation()));
    this.tab = Shuffleboard.getTab("Drivetrain");
    tab.addNumber("Heading", this::getHeading);
    tab.addNumber("X", this::getX);
    tab.addNumber("Y", this::getY);
    tab.addNumber("RPM diff", () -> (getLeftAverageVelocity() - getRightAverageVelocity()));
    tab.addNumber("Amp diff", () -> (getLeftCurrentAmps() - getRightCurrentAmps()));

    // tab.addNumber("Current Speed Limit", this::getSpeedLimit);

    // this.speedLimitWidget = tab.add("Target Flywheel Speed", speedLimit);

    // this.speedLimitWidget
    //     .getEntry()
    //     .addListener(
    //         event -> {
    //           this.setSpeedLimit(event.getEntry().getDouble(this.speedLimit));
    //         },
    //         EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    SmartDashboard.putData("Field", field2d);
    TrajectoryRegister.setField2d(field2d);
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

  // public double getSpeedLimit() {
  //   return speedLimit;
  // }

  // public void setSpeedLimit(double newLimit) {
  //   this.speedLimit = newLimit;
  // }

  /**
   * Drives the robot according to provided DriveMode
   *
   * @param mode
   * @param first Left input in TANK, speed in ARCADE or CURVATURE
   * @param second Right input in TANK, rotation in ARCADE or CURVATURE
   */
  public void driveRobot(DriveMode mode, double first, double second) {
    // Controller interface
    switch (mode) {
      case TANK:
        drive.tankDrive(filter1.calculate(first), filter2.calculate(second));
        break;
      case ARCADE:
        drive.arcadeDrive(filter1.calculate(first), second);
        break;
      case CURVATURE:
        drive.curvatureDrive(filter1.calculate(first), second, true);
        break;
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
    return new DifferentialDriveWheelSpeeds(lEncoder.getVelocity(), rEncoder.getVelocity());
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
    System.out.println("heading change: " + pose.getRotation().getDegrees());
    odometry.resetPosition(pose, getRotation());
    System.out.println("current heading: " + pigeon.getRotation2d().getDegrees());
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

  public Iterable<CANSparkMax> getAllSparks() {
    return allSparks;
  }

  @Override
  public void periodic() {
    updateOdometry();
    field2d.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // driveSim.setInputs(leftSparks[0].getAppliedOutput(),
    // rightSparks[0].getAppliedOutput());
    driveSim.setInputs(leftSparks[0].getAppliedOutput(), rightSparks[0].getAppliedOutput());

    lEncoderSim.setPosition(driveSim.getLeftPositionMeters());
    lEncoderSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond());
    rEncoderSim.setPosition(driveSim.getRightPositionMeters());
    rEncoderSim.setVelocity(driveSim.getRightVelocityMetersPerSecond());
    pigeonSim.setRawHeading(driveSim.getHeading().getDegrees());

    driveSim.update(0.02);
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

    driveSim.setPose(odometry.getPoseMeters());
    field2d.setRobotPose(odometry.getPoseMeters());
  }
}
