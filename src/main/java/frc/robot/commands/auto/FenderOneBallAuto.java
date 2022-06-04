package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.Shoot;
import frc.robot.commands.Turn180;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants.ShooterConstants;

public class FenderOneBallAuto extends SequentialCommandGroup {
    public FenderOneBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, ShooterSubsystem shooter, TurretSubsystem turret) {
        addCommands(
            // new Shoot(
            //     () -> ShooterConstants.FENDER_RPM,

            //     () -> 0,
            //     shooter,
            //     turret,
            //     hopper
            // ),
            new Turn180(drive),
            new DriveRamsete(drive, "DriveOffTarmac", false)
        );
    }
}
