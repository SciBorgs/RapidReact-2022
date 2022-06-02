package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveRamsete;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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
            new DriveRamsete(drive, "Pos_" + pos + "_2Ball_Fender", true)
        );

        // add fender shot code here 
    }
}
