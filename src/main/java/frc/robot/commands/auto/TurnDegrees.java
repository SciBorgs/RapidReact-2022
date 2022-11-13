package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Util;

public class TurnDegrees extends CommandBase {
  private DriveSubsystem drive;
  private double degrees;
  private PIDController turnController;
  private ShuffleboardTab tab;

  public TurnDegrees(double degrees, DriveSubsystem drive) {
    this.drive = drive;
    this.degrees = degrees;
    turnController = DriveConstants.turnPID.get();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    // turnController.enableContinuousInput(0, 359);
    turnController.setTolerance(0.2);
    turnController.setSetpoint(Util.normalizeAngle180(drive.getPoseDegrees() + degrees));
    // turnController.setSetpoint(Util.normalizeAngle360(drive.getHeading() + degrees));
    ;
    // turnController.setSetpoint(drive.getHeading()+180);
    // System.out.println("Desired angle: " + turnController.getSetpoint());
  }

  @Override
  public void execute() {
    double speed = turnController.calculate(drive.getPoseDegrees());
    System.out.println("Speed: " + speed);
    drive.setMotorGroups(-speed, speed);
    System.out.println("Current heading: " + drive.getPoseDegrees());
    System.out.println("Desired heading: " + turnController.getSetpoint());
    System.out.println("At setpoint? " + turnController.atSetpoint());
    // drive.setSpeeds(new DifferentialDriveWheelSpeeds(-speed, speed));
  }

  @Override
  public void end(boolean interrupted) {
    drive.setMotorGroups(0, 0);
    // drive.setSpeeds(new DifferentialDriveWheelSpeeds());
  }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
