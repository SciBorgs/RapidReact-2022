package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.intake.*;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup() {
        addCommands(
            new LowerIntakeArmCommand(),
            new IntakeBallsCommand()
        );
    }
}
