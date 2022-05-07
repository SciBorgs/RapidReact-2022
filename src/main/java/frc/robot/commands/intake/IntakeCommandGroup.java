package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.intake.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(IntakeSubsystem intake, HopperSubsystem hopper) {
        addCommands(
            new LowerIntakeArmCommand(intake),
            new IntakeBallsCommand(intake, hopper)
        );
    }
}
