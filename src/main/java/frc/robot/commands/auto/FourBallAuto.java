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

public class FourBallAuto extends SequentialCommandGroup {
    public FourBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight,
            ShooterSubsystem shooter, TurretSubsystem turret, String initialPos) {

        addCommands(
                new InstantCommand(
                        () -> intake.toggleArm(),
                        intake),
                new InstantCommand(
                        () -> intake.startSuck(),
                        intake));

        addCommands(
                new DriveUntilIntake(drive, intake),
                new Turn180(drive),
                new HighShot(shooter, turret, hopper, limelight)
                        .withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT));

        if (initialPos == "1")
            addCommands(new Turn180(drive));

        addCommands(
                new DriveRamsete(drive, "Pos" + initialPos + "_4Ball", true),
                new Turn180(drive),
                new HighShot(shooter, turret, hopper, limelight)
                        .withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT),
                new InstantCommand(
                        () -> intake.stopSuck(),
                        intake));
    }
}
