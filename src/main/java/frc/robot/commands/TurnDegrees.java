package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.util.Util;

public class TurnDegrees extends CommandBase {
  private DriveSubsystem drive;
  private double degrees;
  private PIDController turnController;

  public TurnDegrees(double degrees, DriveSubsystem drive) {
    this.drive = drive;
    this.degrees = degrees;
    turnController = DriveConstants.turnPID.get();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);
    turnController.setSetpoint(drive.getHeading() + 180);
  }

  @Override
  public void execute() {
    double speed = turnController.calculate(drive.getHeading());
    drive.driveRobot(DriveMode.TANK, -speed, speed);
    System.out.println("Current heading (auto): " + drive.getHeading());
    System.out.println("Desired heading: " + turnController.getSetpoint());
    // drive.setSpeeds(new DifferentialDriveWheelSpeeds(-speed, speed));
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveRobot(DriveMode.TANK, 0, 0);
    // drive.setSpeeds(new DifferentialDriveWheelSpeeds());
  }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
