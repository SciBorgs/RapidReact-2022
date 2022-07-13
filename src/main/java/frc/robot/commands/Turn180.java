package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.Util;

public class Turn180 extends CommandBase {
    private DriveSubsystem drive;
    PIDController turnController;

    public Turn180(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
        turnController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        turnController.enableContinuousInput(-180, 180);
        turnController.setTolerance(2.0);
    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setSetpoint(Util.normalizeAngle180(drive.getHeading()));
    }

    @Override
    public void execute() {
        drive.tankDriveVolts(-turnController.calculate(drive.getHeading()), turnController.calculate(drive.getHeading()));
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }
    public void end(boolean interrupted) {
        drive.tankDriveVolts(0, 0);
    }

}
