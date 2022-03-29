// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.pneumatics.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.AimHoodCommand;
import frc.robot.commands.shooter.ManualHoodCommand;
import frc.robot.commands.turret.AimTurretCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

//for list
import java.util.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi = new OI();
  public static LimeLightSubsystem  limelightSubsystem  = new LimeLightSubsystem();
  public static TurretSubsystem     turretSubsystem     = new TurretSubsystem();
  public static ShooterSubsystem    shooterSubsystem    = new ShooterSubsystem();
  public static IntakeSubsystem     intakeSubsystem     = new IntakeSubsystem();
  public static HopperSubsystem     hopperSubsystem     = new HopperSubsystem();
  public static PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  // public static DriveSubsystem driveSubsystem = new DriveSubsystem();

  private RobotContainer m_robotContainer;

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
  }

  @Override
  public void autonomousInit() { 
    // turretSubsystem.resetPigeon();
    CommandScheduler.getInstance().schedule(new AimTurretCommand());
    // shooterSubsystem.hood_Encoder.setPosition(0);
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // shooterSubsystem.update();
    turretSubsystem.updateShuffleboard();
    shooterSubsystem.update();
  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // System.out.println("This is teleop init");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // new ManualHoodCommand().execute();
    // System.out.println("Angle: " + shooterSubsystem.getHoodAngle());
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