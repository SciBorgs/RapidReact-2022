package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeBallsCommand;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.shooter.ShootSequence.Target;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper,
            LimeLightSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret, String initialPos) {

        addCommands(
                // new TurnToAngle(180, drive),
                new ParallelCommandGroup(
                        new IntakeBallsCommand(intake, hopper),
                        new DriveRamsete(drive, "Pos" + initialPos + "_2Ball")),
                new ShootSequence(shooter, turret, intake, hopper, limelight,
                        Target.HIGH));
    }
}
