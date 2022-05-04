// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.autoProfile.Strategy;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.MonitorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.RumbleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI                    oi                    = new OI(true);

  public static DriveSubsystem        driveSubsystem        = new DriveSubsystem();
  public static LimeLightSubsystem    limelightSubsystem    = new LimeLightSubsystem();
  public static PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();
  public static TurretSubsystem       turretSubsystem       = new TurretSubsystem();
  public static ShooterSubsystem      shooterSubsystem      = new ShooterSubsystem();
  public static IntakeSubsystem       intakeSubsystem       = new IntakeSubsystem();
  public static HopperSubsystem       hopperSubsystem       = new HopperSubsystem();
  public static PneumaticsSubsystem   pneumaticsSubsystem   = new PneumaticsSubsystem();
  public static ClimberSubsystem      climberSubsystem      = new ClimberSubsystem();
  public static MonitorSubsystem      monitorSubsystem      = new MonitorSubsystem();
  public static RumbleSubsystem       RumbleSubsystem       = new RumbleSubsystem(oi.xboxController);

  // public static LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem();

  private SendableChooser<Strategy> autoChooser = AutoProfile.getAutoChooser();
  private Field2d field2d = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // shuffleboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    field2d.setRobotPose(driveSubsystem.getPose());
  }

  // @Override
  // public void simulationInit() {
  //   REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.lFront,  DCMotor.getNEO(1));
  //   REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.lMiddle, DCMotor.getNEO(1));
  //   REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.lBack,   DCMotor.getNEO(1));
  //   REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.rFront,  DCMotor.getNEO(1));
  //   REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.rMiddle, DCMotor.getNEO(1));
  //   REVPhysicsSim.getInstance().addSparkMax(driveSubsystem.rBack,   DCMotor.getNEO(1));
  // }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // AutoProfile.setStrategy(Strategy.TAXI);
    // CommandScheduler.getInstance().schedule(AutoProfile.getAutoCommand());
    // CommandScheduler.getInstance().schedule(new SpinCommand(Math.PI));

    // CommandScheduler.getInstance().schedule(new AutoCommandGroup());
    //limelightSubsystem.setCameraParams(limelightSubsystem.getTable(), "pipeline", 2);
    //double data = limelightSubsystem.getTableData(limelightSubsystem.getTable(), "tx");
    //double data = limeLightSubsystem.getTableData(limeLightSubsystem.getTable(), "pipeline");
    //double data = table.getEntry("tx").getDouble(1.0);
    //System.out.println(data);
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