// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.test.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.DriveCommand;

import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.DummySubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
// import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.util.DelayedPrinter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi = new OI();

  public static DriveSubsystem          driveSubsystem          = new DriveSubsystem();
  public static LocalizationSubsystem   localizationSubsystem   = new LocalizationSubsystem();

  // public static DummySubsystem          dummySubsystem          = new DummySubsystem();
  // public static ShuffleboardSubsystem   shuffleboardSubsystem   = new ShuffleboardSubsystem();

  private RobotContainer m_robotContainer;

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
    this.printer = new DelayedPrinter(1000);

    // shuffleboardSubsystem.bind("test", "test key", dummySubsystem::get4, 0.0);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // localizationSubsystem.update();
    // printer.print(localizationSubsystem.getInfoString());
    // shuffleboardSubsystem.update();
  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void autonomousInit() {
    // TODO: Merge shooter, intake, hopper, ball follow into auto
    // CommandScheduler.getInstance().schedule(
    //   new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       // new StartHopperCommand(),
    //       new MoveToPointAlphaCommand()
    //     ),
    //     // new ShootCommand(),
    //     new MoveToPointBetaCommand(),
    //     // new FollowBallCommand(),
    //     // new IntakeBallCommand(),
    //     new MoveToPointGammaCommand()//,
    //     // new ShootCommand()
    //   )
    // );
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    new DriveCommand().execute();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Robot.driveSubsystem.setSpeed(0, 0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Robot.localizationSubsystem.reset();
  }
}