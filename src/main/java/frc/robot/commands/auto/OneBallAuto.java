package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// fender shot
public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper,
            FlywheelSubsystem flywheel, TurretSubsystem turret) {
        addCommands(
            new InstantCommand(() -> flywheel.setTargetFlywheelSpeed(ShooterConstants.TARMAC_SPEED), flywheel),
            new WaitCommand(ShooterConstants.FLYWHEEL_RAMP_TIMEOUT),
            new InstantCommand(hopper::startElevator, hopper),
            new WaitCommand(ShooterConstants.SINGLE_BALL_TIMEOUT),
            new InstantCommand(hopper::stopElevator, hopper),
            new InstantCommand(flywheel::stopFlywheel, flywheel),
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
                    drive).withTimeout(10),
            new InstantCommand(flywheel::stopFlywheel, flywheel)
        );
    }
}
