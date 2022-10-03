package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Util;

public class TurnDegrees extends CommandBase {
  private DriveSubsystem drive;
  private double degrees;
  private PIDController turnController;

  public TurnDegrees(double degrees, DriveSubsystem drive) {
    this.drive = drive;
    this.degrees = degrees;
    turnController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    turnController.setSetpoint(Util.normalizeAngle180(drive.getHeading(), drive.getHeading()+degrees));
  }

  @Override
  public void execute() {
    double voltage = turnController.calculate(drive.getHeading());
    drive.tankDriveVolts(-voltage, voltage);
  }

  @Override
  public void end(boolean interrupted) {
    drive.tankDriveVolts(0, 0);
  }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
