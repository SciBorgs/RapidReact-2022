package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FenderOneBallAuto extends SequentialCommandGroup {
    public FenderOneBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper,
            ShooterSubsystem shooter, TurretSubsystem turret) {
        addCommands(
                new Shoot(
                        () -> ShooterConstants.FENDER_SPEED,
                        () -> ShooterConstants.FENDER_ANGLE,
                        () -> 0,
                        shooter,
                        turret,
                        hopper),
                new FunctionalCommand(
                        () -> {
                        },
                        () -> {
                            drive.driveRobot(DriveMode.TANK, DriveConstants.driveBackSpeeds,
                                    DriveConstants.driveBackSpeeds);
                        },
                        (interrupted) -> {
                            drive.driveRobot(DriveMode.TANK, 0, 0);
                        },
                        () -> false,
                        drive).withTimeout(10)
        // // new Turn180(drive),
        // // new DriveRamsete(drive, "DriveOffTarmac", false)
        // new RunCommand(() -> {
        // drive.driveBack();
        // }), requirements)
        );
    }
}
