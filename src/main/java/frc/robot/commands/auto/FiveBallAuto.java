package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.HighShot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
    public FiveBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight,
            ShooterSubsystem shooter, TurretSubsystem turret, String initialPos) {

        // init
        addCommands(
                new InstantCommand(
                    () -> intake.toggleArm(),
                    intake
                ),
                new InstantCommand(
                    () -> intake.startSuck(),
                    intake
                ));

        addCommands(
                new HighShot(shooter, turret, hopper, limelight).withTimeout(ShooterConstants.SINGLE_BALL_TIMEOUT), // TODO we can't shoot on tarmac
                new TurnToAngle(180, drive),
                new DriveUntilIntake(drive, intake));

        addCommands(
                new TurnToAngle(0, drive),
                new DriveRamsete(drive, "Pos" + initialPos + "_5Ball_Stage1", true),
                new HighShot(shooter, turret, hopper, limelight).withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT));

        if (initialPos == "2")
            addCommands(new TurnToAngle(180, drive));

        addCommands(
                new DriveRamsete(drive, "Pos" + initialPos + "_5Ball_Stage2", false),
                new TurnToAngle(0, drive),
                new HighShot(shooter, turret, hopper, limelight).withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT),
                new InstantCommand(
                    () -> intake.stopSuck(),
                    intake
                )
        );

    }
}
