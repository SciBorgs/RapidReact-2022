// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.ToggleCompressorCommand;
import frc.robot.commands.FenderShot;
import frc.robot.commands.HighShot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MonitorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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
  public final DriveSubsystem         drive        = new DriveSubsystem();
  public final VisionSubsystem        vision       = new VisionSubsystem();
  public final PhotonVisionSubsystem  photonVision = new PhotonVisionSubsystem();
  public final TurretSubsystem        turret       = new TurretSubsystem();
  public final ShooterSubsystem       shooter      = new ShooterSubsystem();
  public final IntakeSubsystem        intake       = new IntakeSubsystem();
  public final HopperSubsystem        hopper       = new HopperSubsystem();
  public final PneumaticsSubsystem    pneumatics   = new PneumaticsSubsystem();
  public final ClimberSubsystem       climber      = new ClimberSubsystem();
  public final MonitorSubsystem       monitor      = new MonitorSubsystem();
  public final RumbleSubsystem        rumble       = new RumbleSubsystem(oi.xboxController);

  // private final Set<Subsystem> subsystems = Set.of(
  //     drive, vision, photonVision, turret,
  //     shooter, intake, hopper, pneumatics,
  //     climber, monitor, rumble);

  // COMMANDS
  private final ToggleCompressorCommand   toggleCompressorCommand = new ToggleCompressorCommand(pneumatics);
  public  final RumbleCommand             rumbleCommand           = new RumbleCommand(drive, rumble);
  

  // blocker
  // private final Command block = Util.blockSubsystems(subsystems); 

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
    // turret auto following
    // turret.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       if (vision.hasTarget())
    //         turret.setTargetAngle(turret.getCurrentAngle() + vision.getXOffset());
    //       else
    //         turret.setTargetAngle(0);
    //       },
    //     turret));
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    oi.toggleCompressor.whenPressed(toggleCompressorCommand);
    
    // Intake
    // oi.intakeBalls.whenHeld()

    // oi.toggleIntake.whenPressed(
    //   new InstantCommand(
    //     () -> intake.toggleArm()));

    // Intake-Hopper-Compressor
    // oi.startHopper.whenHeld();

    // Climber
    // oi.extendTelescope.whenHeld(
    //   new RunCommand(() -> climber.runTelescope(false), climber)
    // );
    // oi.retractTelescope.whenHeld(
    //   new RunCommand(() -> climber.runTelescope(true), climber)
    // );
    // oi.extendArm.whenHeld(
    //   new RunCommand(() -> climber.runArms(false), climber)
    // );
    // oi.retractArm.whenHeld(
    //   new RunCommand(() -> climber.runArms(true), climber)
    // );

    // Shooter
    oi.shootButton.whenPressed(new HighShot(shooter, turret, hopper, vision).withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT));
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
    // An ExampleCommand will run in autonomous
    // return new TurnToAngle(180, drive);
    // String pathName = "paths/output/Test-Circle.wpilb.json";
    // Trajectory path = TrajectoryUtil.fromPathweaverJson(pathName);
    // return new DriveRamsete(drive, autoChooser.getSelected(), true);
    // return new FiveBallAuto(drive, intake, hopper, vision, shooter, turret, "1");

    // testing shooter
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> hopper.startElevator(0.8), hopper),
    //   new InstantCommand(() -> shooter.setTargetHoodAngle(12), shooter),
    //   new InstantCommand(() -> turret.setTargetAngle(15), turret),
    //   new InstantCommand(() -> shooter.setTargetFlywheelSpeed(8000), shooter)
    // );
    return new FenderShot(shooter, hopper);
  }

  /**
   * sets default commands during teleop
   */
  public void setTeleopCommands() {
    drive.setDefaultCommand(
      new RunCommand(
        () -> drive.driveRobot(
          DriveSubsystem.DriveMode.TANK,
          oi.leftStick.getY(),
          oi.rightStick.getY()),
        drive));
  }
}
