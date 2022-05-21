package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootSequence;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BallCounter;

public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(DriveSubsystem drive, HopperSubsystem hopper, VisionSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret, BallCounter count) {

        addCommands(
            // TODO our limelight may not be able to see the hub from the tarmac
            new ShootSequence(shooter, turret, hopper, limelight, count),
            new DriveUntilIntake(drive, count) // move off tarmac
        );
    }
}
