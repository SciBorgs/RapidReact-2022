// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.Ports.InputDevices;
import frc.robot.Ports.XboxControllerMap;
import frc.robot.commands.SimpleAutos;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MonitorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.DPadButton;
import frc.robot.util.Util;
import frc.robot.util.VisionFilter;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final XboxController xbox = new XboxController(Ports.InputDevices.XBOX_CONTROLLER);
  private final Joystick leftStick = new Joystick(InputDevices.JOYSTICK_LEFT);
  private final Joystick rightStick = new Joystick(InputDevices.JOYSTICK_RIGHT);

  // SUBSYSTEMS
  public final DriveSubsystem drive = new DriveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final MonitorSubsystem monitor = new MonitorSubsystem();

  public final VisionFilter vf = new VisionFilter();

  // AUTO POSITION CHOOSER
  private final SendableChooser<String> positionChooser = Util.getPositionChooser();
  private String currentAutonPositon = "1";

  public void setCurrentAutonPosition(String pos) {
    currentAutonPositon = pos;
  }

  // Auto commands
  private final HashMap<String, Command> autoCommands =
      new HashMap<String, Command>() {
        {
          put("One Ball", SimpleAutos.shootThenDriveBack(drive, flywheel));
          put(
              "Two Ball",
              new TwoBallAuto(drive, intake, hopper, flywheel, turret, currentAutonPositon));
          put(
              "Three Ball",
              new ThreeBallAuto(drive, intake, hopper, flywheel, turret, currentAutonPositon));
          put(
              "Four Ball",
              new FourBallAuto(drive, intake, hopper, flywheel, turret, currentAutonPositon));
          put(
              "Five Ball",
              new FiveBallAuto(drive, intake, hopper, flywheel, turret, currentAutonPositon));
          put(
              "Fender Two Ball",
              new FenderTwoBallAuto(drive, intake, hopper, flywheel, turret, currentAutonPositon));
          put(
              "Fender Three Ball",
              new FenderThreeBallAuto(
                  drive, intake, hopper, flywheel, turret, currentAutonPositon));
        }
      };

  // AUTO COMMAND CHOOSER
  private final SendableChooser<Command> autoCommandChooser = Util.getAutoChooser(autoCommands);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // turret.setDefaultCommand(
    //     new RunCommand(
    //         () -> turret.setTargetAngle(turret.getCurrentAngle() + vf.getXOffset()), turret));

    // hood auto aiming
    hood.setDefaultCommand(
        new RunCommand(
            () -> hood.setSetpoint(ShooterConstants.getHoodAngle(vf.getDistance())), hood));
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
        // .and(new Trigger(hood::atSetpoint))
        .and(new Trigger(turret::atTarget))
        .debounce(0.3, DebounceType.kFalling)
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
        // .whileHeld(
        //     () -> flywheel.setTargetFlywheelSpeed(ShooterConstants.getRPM(vf.getDistance())),
        //     flywheel)
        .whenPressed(() -> flywheel.setTargetFlywheelSpeed(ShooterConstants.TARMAC_RPM))
        .whenReleased(flywheel::stopFlywheel, flywheel);

    // Run flywheel at set speed
    new JoystickButton(xbox, XboxControllerMap.Button.BUMPER_LEFT)
        .whenPressed(() -> flywheel.setTargetFlywheelSpeed(ShooterConstants.FENDER_RPM), flywheel)
        .whenReleased(flywheel::stopFlywheel, flywheel);
  }

  public SendableChooser<Command> getAutoCommandChooser() {
    return this.autoCommandChooser;
  }

  public SendableChooser<String> getPositionChooser() {
    return this.positionChooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return SimpleAutos.shootThenDriveBack(drive, flywheel);
  }
}
