package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallsCommand extends ParallelCommandGroup {
    public IntakeBallsCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
        addCommands(
            new StartEndCommand(() -> intake.startSuck(), () -> intake.stopSuck(), intake),
            new StartEndCommand(() -> hopper.startSuck(), () -> hopper.stopSuck(), hopper)
        );
    }
}