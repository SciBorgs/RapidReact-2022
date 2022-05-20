package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeBallsCommandGroup;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.shooter.ShootSequence.Target;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
    public FourBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret, String initialPos) {
        
        addCommands(
            new ParallelRaceGroup(
                new IntakeBallsCommandGroup(intake, hopper),
                new DriveUntilIntake(drive, intake)
            ),
            new TurnToAngle(180, drive),
            new ShootSequence(shooter, turret, intake, hopper, limelight, Target.HIGH)
        );

        if(initialPos == "1") addCommands(new TurnToAngle(0, drive));
        
        addCommands(
            new ParallelRaceGroup(
                new IntakeBallsCommandGroup(intake, hopper),
                new DriveRamsete(drive, "Pos" + initialPos + "_4Ball")
            ),
            new TurnToAngle(180, drive),
            new ShootSequence(shooter, turret, intake, hopper, limelight, Target.HIGH)
        );

    }
}
