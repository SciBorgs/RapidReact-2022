package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FenderShot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * fender shot
 */
public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(DriveSubsystem drive, HopperSubsystem hopper, IntakeSubsystem intake, ShooterSubsystem shooter) {

        addCommands(
                new FenderShot(shooter, hopper),
                new DriveUntilIntake(drive, intake) // TODO move off tarmac backwards
        );
    }
}
