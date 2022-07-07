// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap.InputDevices;
import frc.robot.PortMap.XboxControllerMap;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.Shoot;
import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.auto.OneBallAuto;
import frc.robot.commands.auto.ThreeBallAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MonitorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.util.DPadButton;
import frc.robot.util.Util;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final XboxController xbox = new XboxController(PortMap.InputDevices.XBOX_CONTROLLER);
  private final Joystick leftStick = new Joystick(InputDevices.JOYSTICK_LEFT);
  private final Joystick rightStick = new Joystick(InputDevices.JOYSTICK_RIGHT);

  // SUBSYSTEMS
  private final DriveSubsystem drive = new DriveSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    vision.reset();
    configureButtonBindings();
    configureSubsystemDefaults();
  }

  private void configureSubsystemDefaults() {
    // turret auto aiming
    turret.setDefaultCommand(
      new RunCommand(
        () -> turret.setTargetAngle(turret.getCurrentAngle() + vision.getXOffset()),
        turret));
    
    // hood auto aiming
    hood.setDefaultCommand(
      new RunCommand(
        () -> hood.setSetpoint(ShooterConstants.getHoodAngle(vision.getDistance())),
        hood));
    hood.enable();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Compressor
    new JoystickButton(xbox, XboxControllerMap.Button.START)
        .toggleWhenPressed(
            new StartEndCommand(
                pneumatics::start,
                pneumatics::stop,
                pneumatics));

    // Intake
    new JoystickButton(xbox, XboxControllerMap.Button.X)
        .whileHeld(
            new StartEndCommand(
                () -> {
                  intake.startSuck();
                  hopper.startSuck();
                },
                () -> {
                  intake.stopSuck();
                  hopper.stopSuck();
                },
                intake, hopper))
        .whenPressed(
          new InstantCommand(
            intake::toggleArm,
            intake));
    
    new JoystickButton(xbox, 2) // TODO update with proper port
        .whileHeld(
          new StartEndCommand(
            () -> {
              intake.reverseSuck();
              hopper.reverseSuck();
            },
            () -> {
              intake.stopSuck();
              hopper.stopSuck();
            },
            intake, hopper));

    new JoystickButton(xbox, XboxControllerMap.Button.BACK)
      .whenPressed(
        new InstantCommand(
            intake::toggleArm,
            intake));

    // Intake-Hopper-Compressor
    new JoystickButton(xbox, XboxControllerMap.Button.A)
        .whileHeld(
            new StartEndCommand(
                () -> {
                  hopper.startSuck();
                  hopper.startElevator();
                },
                () -> {
                  hopper.stopSuck();
                  hopper.stopElevator();
                },
                hopper));

    // Climber
    new JoystickButton(xbox, XboxControllerMap.Button.Y)
        .whileHeld(
          new StartEndCommand(
            climber::extendTelescope,
            climber::stopTelescope,
            climber));

    new JoystickButton(xbox, XboxControllerMap.Button.B)
        .whileHeld(
          new StartEndCommand(
            climber::retractTelescope,
            climber::stopTelescope,
            climber));
            
    new DPadButton(xbox, DPadButton.Direction.LEFT)
        .whileHeld(
          new StartEndCommand(
            climber::extendArms,
            climber::stopArms,
            climber));

    new DPadButton(xbox, DPadButton.Direction.RIGHT)
        .whileHeld(
          new StartEndCommand(
            climber::retractArms,
            climber::stopArms,
            climber));

    // Shooter
    new JoystickButton(xbox, XboxControllerMap.Button.BUMPER_RIGHT)
      .whenPressed(
        new Shoot(flywheel, hopper, vision));

    new JoystickButton(xbox, XboxControllerMap.Button.BUMPER_LEFT)
      .whileHeld(
        new StartEndCommand(
          () -> flywheel.setTargetFlywheelSpeed(ShooterConstants.TARMAC_RPM),
          flywheel::stopFlywheel,
          flywheel));

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
    // return new RunCommand(() -> drive.driveRobot(DriveMode.TANK, 0.7, 0.7), drive);
    return new DriveRamsete(drive, "Pos1_5Ball_Stage1", true);
    // return new FiveBallAuto(drive, intake, hopper, vision, flywheel, turret, "1");
    // return new InstantCommand();
  }

  /**
   * sets default commands during teleop
   */
  public void setTeleopCommands() {
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              drive.driveRobot(
                  DriveSubsystem.DriveMode.TANK,
                  leftStick.getY(),
                  rightStick.getY());
            },
            drive));
            
    // rumble.setDefaultCommand(
    // new ConditionalCommand(
    // new InstantCommand(rumble::rumble, rumble),
    // new InstantCommand(rumble::stopRumble, rumble),
    // drive::isStalling));
  }
}
