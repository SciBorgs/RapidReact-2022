package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.shooter.ShootSequence.Target;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret) {
        
        addCommands(
            new ShootSequence(shooter, turret, intake, hopper, limelight, Target.HIGH),
            new DriveUntilIntake(drive, intake)
        );
    }
}
