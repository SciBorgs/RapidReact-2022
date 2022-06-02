package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants.ShooterConstants;

public class FenderTwoBallAuto extends SequentialCommandGroup {
    public FenderTwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, ShooterSubsystem shooter, TurretSubsystem turret, String pos) {
        addCommands(
            new InstantCommand(
                () -> intake.toggleArm(),
                intake
            ),
            new InstantCommand(
                () -> intake.startSuck(),
                intake
            )
        );

        addCommands(
            new DriveRamsete(drive, "Pos_" + pos + "_2Ball_Fender", true),
            new Shoot(
                () -> ShooterConstants.FENDER_RPM,
                () -> ShooterConstants.FENDER_ANGLE,
                () -> 0,
                shooter,
                turret,
                hopper
            )
        );

        // add fender shot code here 
    }
}
