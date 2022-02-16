// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.test.MoveToPointTestCommand;
import frc.robot.commands.test.PatrolTestCommand;
import frc.robot.commands.test.SpinTestCommand;
import frc.robot.commands.DriveCommand;
// import frc.robot.commands.FollowBallCommand;
// import frc.robot.commands.IntakeBallCommand;
// import frc.robot.commands.StartHopperCommand;
import frc.robot.commands.shooter.ShootCommand;
// import frc.robot.commands.turret.AimTurretCommand;
import frc.robot.commands.auto.MoveToPointAlphaCommand;
import frc.robot.commands.auto.MoveToPointBetaCommand;
import frc.robot.commands.auto.MoveToPointGammaCommand;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.util.DelayedPrinter;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi = new OI();

  // public static LimeLightSubsystem      limelightSubsystem      = new LimeLightSubsystem();
  // public static TurretSubsystem         turretSubsystem         = new TurretSubsystem();
  // public static ShooterSubsystem        shooterSubsystem        = new ShooterSubsystem();
  public static DriveSubsystem          driveSubsystem = new DriveSubsystem();
  public static LocalizationSubsystem   localizationSubsystem   = new LocalizationSubsystem();

  private RobotContainer m_robotContainer;
  // public static IntakeSubsystem intake = new IntakeSubsystem();
  // public static HopperSubsystem hopper = new HopperSubsystem();
  // public static PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();

  private DelayedPrinter printer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //table = NetworkTableInstance.getDefault().getTable("limelight");
    this.printer = new DelayedPrinter(100);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //limelightSubsystem.setCameraParams(limelightSubsystem.getTable(), "pipeline", 2);
    //double data = limelightSubsystem.getTableData(limelightSubsystem.getTable(), "tx");
    //double data = limeLightSubsystem.getTableData(limeLightSubsystem.getTable(), "pipeline");
    //double data = table.getEntry("tx").getDouble(1.0);
    //System.out.println(data);

    localizationSubsystem.update();
    printer.print(localizationSubsystem.getInfoString());
  }

  @Override
  public void autonomousInit() {
    System.out.println("This is autonomous init");

    /*
    CommandScheduler.getInstance().schedule(
      new SequentialCommandGroup(
        new MoveToPointAlphaCommand(),
        new ShootCommand(),
        new MoveToPointBetaCommand(),
        new FollowBallCommand(),
        new IntakeBallCommand(),
        new StartHopperCommand(),
        new MoveToPointGammaCommand(),
        new ShootCommand()
      )
    );
    */

     CommandScheduler.getInstance().schedule(
      //  new MoveToPointBetaCommand()
      // new PatrolTestCommand()
      new SpinTestCommand()
     );
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // System.out.println("This is teleop init");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Robot.localizationSubsystem.reset();
    Robot.localizationSubsystem.update();
    // new DriveCommand().execute();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // System.out.println("This is disabled init");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // System.out.println("This is disabled periodic");
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // System.out.println("This is test init");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // System.out.println("This is test periodic");
  }
}