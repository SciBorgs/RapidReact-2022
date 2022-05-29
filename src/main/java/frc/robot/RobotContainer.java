// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.auto.DriveRamsete;
import frc.robot.subsystems.DriveSubsystem;
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
  // CONTROLLERS
  private final OI oi = new OI(false);

  // SUBSYSTEMS
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  // public final IntakeSubsystem ntakeSubsystem = new IntakeSubsystem();

  // AUTO CHOOSER
  private final SendableChooser<String> autoChooser = Util.getPathTestChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // drives robot in tank drive according to the joysticks
    if(Robot.isReal()) {
      driveSubsystem.setDefaultCommand(new RunCommand(
        () -> driveSubsystem.driveRobot(
            DriveSubsystem.DriveMode.TANK,
            oi.leftStick.getY(),
            oi.rightStick.getY()),
        driveSubsystem));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() { }

  public SendableChooser<String> getAutoChooser() {
    return this.autoChooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 
    // An ExampleCommand will run in autonomous
    // return new TurnToAngle(180, driveSubsystem);
    // String pathName = "paths/output/Test-Circle.wpilb.json";
    // Trajectory path = TrajectoryUtil.fromPathweaverJson(pathName);
    return new DriveRamsete(driveSubsystem, autoChooser.getSelected(), true);
    // return new FiveBallAuto(driveSubsystem, intakeSubsystem, hopperSubsystem, limelightSubsystem, shooterSubsystem, turretSubsystem, "1");
  }
}
