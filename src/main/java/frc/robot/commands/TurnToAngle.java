package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;


public class TurnToAngle extends PIDCommand {
    public TurnToAngle(double targetDegrees, DriveSubsystem drive) {
        super(
                new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
                drive::getHeading,
                targetDegrees,
                output -> drive.driveRobot(DriveMode.TANK, -output, output),
                drive);

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(0.2);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}
