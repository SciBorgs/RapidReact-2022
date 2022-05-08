// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.commands.climber.RunArmCommand;
import frc.robot.commands.climber.RunTelescopeCommand;
import frc.robot.commands.intake.IntakeBallsCommand;
import frc.robot.commands.shooter.AimTurretCommand;
import frc.robot.commands.shooter.ResetTurretCommand;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.shooter.ShootSequence.Target;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.util.Util;

import static frc.robot.PortMap.*;
import static frc.robot.PortMap.Joystick.*;
import static frc.robot.PortMap.XboxController.*;

import java.util.Set;

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
  private final OI oi = new OI(true);

  // SUBSYSTEMS
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
  public final RumbleSubsystem       rumbleSubsystem       = new RumbleSubsystem(oi.xboxController);

  private final Set<Subsystem> subsystems = Set.of(
      driveSubsystem, limelightSubsystem, photonVisionSubsystem, turretSubsystem,
      shooterSubsystem, intakeSubsystem, hopperSubsystem, pneumaticsSubsystem,
      climberSubsystem, monitorSubsystem, rumbleSubsystem);

  // COMMANDS
  // climber

  // drive
  // not sure what's going on with drive commands right now

  // hopper

  // intake
  private final IntakeBallsCommand intakeBallsCommand = new IntakeBallsCommand(intakeSubsystem, hopperSubsystem);
  // pneumatics
  // shooter
  private final ShootSequence shootSequence = new ShootSequence(shooterSubsystem, turretSubsystem, hopperSubsystem, limelightSubsystem, Target.HIGH);
  // turret
  private final AimTurretCommand aimTurretCommand = new AimTurretCommand(turretSubsystem, limelightSubsystem);
  private final ResetTurretCommand resetTurretCommand = new ResetTurretCommand(turretSubsystem);

  // blocker
  private final Command block = Util.blockSubsystems(subsystems);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // drives robot in tank drive according to the joysticks
    driveSubsystem.setDefaultCommand(new RunCommand(
        () -> driveSubsystem.driveRobot(
            DriveMode.TANK,
            oi.leftStick.getY(),
            oi.rightStick.getY()),
        driveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake
    // this.intakeBalls.whenHeld(new IntakeBallsCommand());
    // this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());
    // this.retractIntakeArms.whenPressed(new RetractIntakeArmCommand());

    // Intake-Hopper-Compressor
    // this.startHopper.whenHeld(new StartHopperCommand());
    // this.hopper.whileHeld(new StartHopperCommand());

    // Climber
    oi.extendTelescope.whenHeld(
      new RunCommand(() -> climberSubsystem.runTelescope(false), climberSubsystem)
    );
    oi.retractTelescope.whenHeld(
      new RunCommand(() -> climberSubsystem.runTelescope(true), climberSubsystem)
    );
    oi.extendArm.whenHeld(
      new RunCommand(() -> climberSubsystem.runArms(false), climberSubsystem)
    );
    oi.retractArm.whenHeld(
      new RunCommand(() -> climberSubsystem.runArms(true), climberSubsystem)
    );

    // Shooter
    oi.shootButton.whenPressed(shootSequence);
  }

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
