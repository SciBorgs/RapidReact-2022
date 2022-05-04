// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.commands.turret.AimTurretCommand;
import frc.robot.commands.turret.ResetTurretCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import static frc.robot.PortMap.*;
import static frc.robot.PortMap.Joystick.*;
import static frc.robot.PortMap.XboxController.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // CONTROLLERS
  public final XboxController xboxController = new XboxController(XBOX_CONTROLLER);
  public final Joystick leftStick = new Joystick(JOYSTICK_LEFT);
  public final Joystick rightStick = new Joystick(JOYSTICK_RIGHT);

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
  public final RumbleSubsystem       rumbleSubsystem       = new RumbleSubsystem(xboxController);
  
  // COMMANDS
  // climber
  // drive
  // hopper
  // intake
  // pneumatics
  // shooter
  // turret
  public final AimTurretCommand      aimTurretCommand      = new AimTurretCommand(turretSubsystem, limelightSubsystem);
  public final ResetTurretCommand    resetTurretCommand    = new ResetTurretCommand(turretSubsystem);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // drives robot in tank drive according to the joysticks
    driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> driveSubsystem.driveRobot(
          DriveMode.TANK,
          leftStick,
          rightStick)
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Climber
    // this.extendTelescope  = new JoystickButton(this.xboxController, XBOX_Y);
    // this.retractTelescope = new JoystickButton(this.xboxController, XBOX_A);
    // this.extendArm  = new JoystickButton(this.xboxController, XBOX_X);
    // this.retractArm = new JoystickButton(this.xboxController, XBOX_B);


    // Intake
    // this.intakeBalls.whenHeld(new IntakeBallsCommand());
    // this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());
    // this.retractIntakeArms.whenPressed(new RetractIntakeArmCommand());

    // Intake-Hopper-Compressor
    // this.startHopper.whenHeld(new StartHopperCommand());
    // this.hopper.whileHeld(new StartHopperCommand());

    // Climber
    // this.extendTelescope.whenHeld(new RunTelescopeCommand(false));
    // this.retractTelescope.whenHeld(new RunTelescopeCommand(true));
    // this.extendArm.whenHeld(new RunArmCommand(false));
    // this.retractArm.whenHeld(new RunArmCommand(true));

    // Shooter
    // this.aimButton.whenPressed(new AimCommandGroup());
    // this.shootButton.whenPressed(new ShootSequence());

    // this.lowerHoodButton.whenHeld(new LowerHoodCommand());
    // this.raiseHoodButton.whenHeld(new RaiseHoodCommand());
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
