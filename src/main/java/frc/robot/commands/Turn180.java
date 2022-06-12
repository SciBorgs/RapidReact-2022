package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.util.Util;

public class Turn180 extends PIDCommand {
    public Turn180(DriveSubsystem drive) {
        super(
                new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
                drive::getHeading,
                Util.normalizeAngle180(drive.getHeading()),
                // If current heading is less than the desired angle, we spin right. Otherwise, we spin left.
                output -> drive.driveRobot(DriveMode.TANK, -output, output),
                        // drive.getHeading() < Util.normalizeAngle180(drive.getHeading()) ? output : -output, 
                        // drive.getHeading() < Util.normalizeAngle180(drive.getHeading()) ? -output : output),
                drive
            );

        System.out.println("NORMALIZED ANGLE: " + Util.normalizeAngle180(drive.getHeading()));

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(4);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
