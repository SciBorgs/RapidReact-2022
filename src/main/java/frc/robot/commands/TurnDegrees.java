package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Util;

public class TurnDegrees extends CommandBase {
  private DriveSubsystem drive;
  private double degrees;
  private PIDController turnController;
  private SimpleMotorFeedforward feedforward;

  public TurnDegrees(double degrees, DriveSubsystem drive) {
    this.drive = drive;
    this.degrees = degrees;
    turnController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    feedforward =
        new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(0.2);
    turnController.setSetpoint(Util.normalizeAngle180(drive.getHeading(), degrees));
  }

  @Override
  public void execute() {
    double fb = turnController.calculate(drive.getHeading());
    double ff = feedforward.calculate(drive.getHeading());
    double voltage = fb + ff;
    drive.setMotorGroups(-voltage, voltage);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setMotorGroups(0, 0);
  }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}