package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.subsystems.FlywheelSubsystem;

public final class SimpleAutos {
  public static Command shootThenDriveBack(DriveSubsystem drive, FlywheelSubsystem flywheel) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> flywheel.setTargetFlywheelSpeed(ShooterConstants.TARMAC_RPM), flywheel),
        new WaitCommand(ShooterConstants.FLYWHEEL_RAMP_TIMEOUT),
        new InstantCommand(flywheel::stopFlywheel, flywheel),
        // new RunCommand(() -> drive.setMotorGroups(DriveConstants.driveBackSpeeds,
        // DriveConstants.driveBackSpeeds), drive)
        new RunCommand(
                () ->
                    drive.driveRobot(
                        DriveMode.TANK,
                        DriveConstants.driveBackSpeeds,
                        DriveConstants.driveBackSpeeds),
                drive)
            .withTimeout(10),
        new InstantCommand(() -> drive.driveRobot(DriveMode.TANK, 0, 0), drive));
  }
}
