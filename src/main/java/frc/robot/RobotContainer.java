// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap.InputDevices;
import frc.robot.PortMap.XboxControllerMap;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MonitorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.DPadButton;
import frc.robot.util.Util;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final XboxController xbox = new XboxController(PortMap.InputDevices.XBOX_CONTROLLER);
  private final Joystick leftStick = new Joystick(InputDevices.JOYSTICK_LEFT);
  private final Joystick rightStick = new Joystick(InputDevices.JOYSTICK_RIGHT);

  // SUBSYSTEMS
  public final DriveSubsystem drive = new DriveSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final MonitorSubsystem monitor = new MonitorSubsystem();
  private final RumbleSubsystem rumble = new RumbleSubsystem(xbox);

  // AUTO CHOOSER
  private final SendableChooser<String> autoChooser = Util.getPathTestChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    vision.reset();
    configureButtonBindings();
    configureSubsystemDefaults();
  }

  private void configureSubsystemDefaults() {
    // drive
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              drive.driveRobot(
                  DriveSubsystem.DriveMode.TANK, -leftStick.getY(), -rightStick.getY());
            },
            drive));

    // turret auto aiming
    turret.setDefaultCommand(
        new RunCommand(
            () -> turret.setTargetAngle(turret.getCurrentAngle() + vision.getXOffset()), turret));

    // hood auto aiming
    hood.setDefaultCommand(
        new RunCommand(
            () -> hood.setSetpoint(ShooterConstants.getHoodAngle(vision.getDistance())), hood));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Shoot trigger for when conditions are met
    new Trigger(flywheel::atTargetRPM)
        .and(new Trigger(() -> flywheel.getTargetFlywheelSpeed() != 0))
        .and(new Trigger(hood::atSetpoint))
        .and(new Trigger(turret::atTarget))
        .debounce(0.2)
        .whenActive(hopper::startElevator, hopper)
        .whenInactive(hopper::stopElevator, hopper);

    // Compressor
    new JoystickButton(xbox, XboxControllerMap.Button.START)
        .toggleWhenPressed(new StartEndCommand(pneumatics::start, pneumatics::stop, pneumatics));

    // Intake balls
    new JoystickButton(xbox, XboxControllerMap.Button.X)
        .whenPressed(intake::extendArm, intake)
        .whileHeld(
            () -> {
              intake.startSuck();
              hopper.startSuck();
            },
            intake,
            hopper)
        .whenReleased(
            () -> {
              intake.stopSuck();
              hopper.stopSuck();
              intake.retractArm();
            },
            intake,
            hopper);

    // Reverse intake
    new JoystickButton(xbox, XboxControllerMap.Button.A)
        .whenPressed(
            () -> {
              intake.reverseSuck();
              hopper.reverseSuck();
            },
            intake,
            hopper)
        .whenReleased(
            () -> {
              intake.stopSuck();
              hopper.stopSuck();
            },
            intake,
            hopper);

    // Toggle arm
    new JoystickButton(xbox, XboxControllerMap.Button.BACK).whenPressed(intake::toggleArm, intake);

    // Run hopper
    new JoystickButton(xbox, XboxControllerMap.Button.A)
        .whenPressed(
            () -> {
              hopper.startSuck();
              hopper.startElevator();
            },
            hopper)
        .whenReleased(
            () -> {
              hopper.stopSuck();
              hopper.stopElevator();
            },
            hopper);

    // Climber
    new JoystickButton(xbox, XboxControllerMap.Button.Y)
        .whenPressed(climber::extendTelescope, climber)
        .whenReleased(climber::stopTelescope, climber);

    new JoystickButton(xbox, XboxControllerMap.Button.B)
        .whenPressed(climber::retractTelescope, climber)
        .whenReleased(climber::stopTelescope, climber);

    new DPadButton(xbox, DPadButton.Direction.LEFT)
        .whenPressed(climber::extendArms, climber)
        .whenReleased(climber::stopArms, climber);

    new DPadButton(xbox, DPadButton.Direction.RIGHT)
        .whenPressed(climber::retractArms, climber)
        .whenReleased(climber::stopArms, climber);

    // Run flywheel at variable speed
    new JoystickButton(xbox, XboxControllerMap.Button.BUMPER_RIGHT)
        .whileHeld(
            () -> flywheel.setTargetFlywheelSpeed(ShooterConstants.getRPM(vision.getDistance())),
            flywheel)
        .whenReleased(flywheel::stopFlywheel, flywheel);

    // Run flywheel at set speed
    new JoystickButton(xbox, XboxControllerMap.Button.BUMPER_LEFT)
        .whenPressed(() -> flywheel.setTargetFlywheelSpeed(ShooterConstants.TARMAC_RPM), flywheel)
        .whenReleased(flywheel::stopFlywheel, flywheel);
  }

  public SendableChooser<String> getAutoChooser() {
    return this.autoChooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // String pathName = "paths/output/Test-Circle.wpilb.json";
    // Trajectory path = TrajectoryUtil.fromPathweaverJson(pathName);
    // // return new RunCommand(() -> drive.driveRobot(DriveMode.TANK, 0.7, 0.7), drive);
    // return new DriveRamsete(drive, "DriveOffTarmac", true);
    return new TurnDegrees(-20, drive);
    // return new Turn180(drive);
    // return new FiveBallAuto(drive, intake, hopper, vision, flywheel, turret, "1");
    // return new InstantCommand();
  }
}
