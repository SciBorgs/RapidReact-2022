// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import frc.robot.subsystems.*;
import frc.robot.subsystems.localization.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;

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

  public static DriveSubsystem        driveSubsystem        = new DriveSubsystem();
  public static LimeLightSubsystem    limelightSubsystem    = new LimeLightSubsystem();
  public static PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();
  public static TurretSubsystem       turretSubsystem       = new TurretSubsystem();
  public static ShooterSubsystem      shooterSubsystem      = new ShooterSubsystem();
  public static IntakeSubsystem       intakeSubsystem       = new IntakeSubsystem();
  public static HopperSubsystem       hopperSubsystem       = new HopperSubsystem();
  public static PneumaticsSubsystem   pneumaticsSubsystem   = new PneumaticsSubsystem();
  public static ClimberSubsystem      climberSubsystem      = new ClimberSubsystem();

  public static NetworkTableSubsystem networkTableSubsystem = new NetworkTableSubsystem();
  public static LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem();


  private Field2d field2d = new Field2d();


  private ShuffleboardTab mainTab;
  private NetworkTableEntry hopperGetSuck;
  private NetworkTableEntry hopperSetSuck;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // localization
    networkTableSubsystem.bind("localization", "pose", localizationSubsystem::get, new double[] {0, 0, 0});
    networkTableSubsystem.bind("localization", "particles", localizationSubsystem.particleFilter::getFlat, new double[] {});
    networkTableSubsystem.bind("localization", "meanParticle", localizationSubsystem.particleFilter::getMeanParticle, new double[] {8, 4});

    // teleop sim
    networkTableSubsystem.bind("drive", "joyLeft", oi.leftStick::getY, 0.0);
    networkTableSubsystem.bind("drive", "joyRight", oi.rightStick::getY, 0.0);
    networkTableSubsystem.bind("drive", "driveSpeed", localizationSubsystem::getVel, 0.0);
    networkTableSubsystem.bind("drive", "driveLimit", driveSubsystem::setSpeedLimit, 1.0);

    // auto sequence
    networkTableSubsystem.bind("auto", "profile", AutoProfile::updateTransportProfile, "TEST");

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
    shooterSubsystem.update();

    turretSubsystem.updateShuffleboard();

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
    CommandScheduler.getInstance().schedule(AutoProfile.getAutoCommand());
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