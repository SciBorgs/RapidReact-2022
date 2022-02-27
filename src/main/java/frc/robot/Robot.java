// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.test.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.DriveCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.NetworkTableSubsystem;

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

  public static NetworkTableSubsystem   networkTableSubsystem   = new NetworkTableSubsystem();

  private RobotContainer m_robotContainer;

  private Field2d field2d = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // localization
    networkTableSubsystem.bind("localization", "rX", localizationSubsystem::getX, 0.0);
    networkTableSubsystem.bind("localization", "rY", localizationSubsystem::getY, 0.0);
    networkTableSubsystem.bind("localization", "rH", localizationSubsystem::getHeading, 0.0);

    // drive
    networkTableSubsystem.bind("drive", "vLimit", driveSubsystem::setSpeedLimit, 1.0);

    SmartDashboard.putData("Field", field2d);

    System.out.println(networkTableSubsystem);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    localizationSubsystem.update();
    networkTableSubsystem.update();
    field2d.setRobotPose(localizationSubsystem.getX(), localizationSubsystem.getY(), new Rotation2d(localizationSubsystem.getHeading()));
  }

  @Override
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.lFront,  DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.lMiddle, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.lBack,   DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.rFront,  DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.rMiddle, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.rBack,   DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // TODO: Merge shooter, intake, hopper, ball follow into auto
    CommandScheduler.getInstance().schedule(
      new SequentialCommandGroup(
        new MoveToPointAlphaCommand(),
        new MoveToPointBetaCommand(),
        new CommandBase() {
          @Override
          public boolean isFinished() {
            return true;
          }
          @Override
          public void end(boolean i) {
            System.out.println("Auto Sequence Completed!");
          }
        },
        new PatrolTestCommand()
      )
    );

    // CommandScheduler.getInstance().schedule(
    //   // new PatrolTestCommand()
    //   // new MoveToPointAlphaCommand()
    //   new AlongAxisTestCommand()
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
    if (Robot.isReal())
      new DriveCommand().execute();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    driveSubsystem.setSpeed(0.0, 0.0);
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
    Robot.localizationSubsystem.reset();
  }
}