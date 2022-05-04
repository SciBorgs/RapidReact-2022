// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final OI                    oi                    = new OI(true);

  public final DriveSubsystem        driveSubsystem        = new DriveSubsystem();
  public final LimeLightSubsystem    limelightSubsystem    = new LimeLightSubsystem();
  public final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();
  public final TurretSubsystem       turretSubsystem       = new TurretSubsystem();
  public final ShooterSubsystem      shooterSubsystem      = new ShooterSubsystem();
  public final IntakeSubsystem       intakeSubsystem       = new IntakeSubsystem();
  public final HopperSubsystem       hopperSubsystem       = new HopperSubsystem();
  public final PneumaticsSubsystem   pneumaticsSubsystem   = new PneumaticsSubsystem();
  public final ClimberSubsystem      climberSubsystem      = new ClimberSubsystem();
  public final MonitorSubsystem      monitorSubsystem      = new MonitorSubsystem();
  public final RumbleSubsystem       RumbleSubsystem       = new RumbleSubsystem(oi.xboxController);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
//   private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return AutoProfile.getAutoCommand();
  }
}
