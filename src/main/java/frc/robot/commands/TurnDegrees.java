package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
    turnController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(0.2);
    turnController.setSetpoint(
        Util.normalizeAngle180(drive.getHeading(), degrees));
  }

  @Override
  public void execute() {
    double voltage = turnController.calculate(drive.getHeading());
    // System.out.println(voltage);
    drive.driveRobot(DriveMode.TANK, -voltage, voltage);
  }

  // @Override
  // public void end(boolean interrupted) {
  //   System.out.println("yea");
  //   drive.driveRobot(DriveMode.TANK, 0, 0);
  // }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
