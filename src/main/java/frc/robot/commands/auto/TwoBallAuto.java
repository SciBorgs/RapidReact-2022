package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.HighShot;
import frc.robot.commands.intake.IntakeForever;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.intake.ToggleIntakeArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight,
            ShooterSubsystem shooter, TurretSubsystem turret) {

        addCommands(
                new ToggleIntakeArm(intake),
                new IntakeForever(intake));

        addCommands(
                new DriveUntilIntake(drive, intake),
                new TurnToAngle(180, drive),
                new HighShot(shooter, turret, hopper, limelight)
                        .withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT),
                new IntakeStop(intake));
    }
}
