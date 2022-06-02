package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;
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
                new Shoot(shooter, hopper).withTimeout(ShooterConstants.DOUBLE_BALL_TIMEOUT),
                new Turn180(drive),
                new DriveRamsete(drive, "DriveOffTarmac", true)
        );
    }
}
