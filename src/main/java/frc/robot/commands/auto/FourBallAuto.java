package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.intake.IntakeForever;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.intake.ToggleIntakeArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BallCounter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
    public FourBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret, BallCounter count, String initialPos) {
        
        addCommands(
            new ToggleIntakeArm(intake),
            new IntakeForever(intake)
        );

        addCommands(
            new DriveUntilIntake(drive, count),
            new TurnToAngle(180, drive),
            new ShootSequence(shooter, turret, hopper, limelight, count)
        );

        if(initialPos == "1") addCommands(new TurnToAngle(0, drive));
        
        addCommands(
            new DriveRamsete(drive, "Pos" + initialPos + "_4Ball", true),
            new TurnToAngle(180, drive),
            new ShootSequence(shooter, turret, hopper, limelight, count),
            new IntakeStop(intake)
        );

    }
}
