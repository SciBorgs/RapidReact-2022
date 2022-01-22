// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.commands.DriveCommand;
import frc.robot.controllers.Following;

public class Robot extends TimedRobot {
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static Following following = new Following();

  public static OI oi = new OI();

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    System.out.println("This is robot periodic");
  }

  @Override
  public void autonomousInit() {
    System.out.println("This is autonomous init");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("This is teleop init");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    new DriveCommand().execute();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("This is disabled init");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    System.out.println("This is disabled periodic");
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("This is test init");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    System.out.println("This is test periodic");
  }
}