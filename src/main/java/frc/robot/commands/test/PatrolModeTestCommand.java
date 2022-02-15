package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PatrolModeTestCommand extends SequentialCommandGroup {
    public PatrolModeTestCommand() {
        super(
            new PatrolPointUnoTestCommand()
            // new PatrolPointUnoTestCommand(),
            // new PatrolPointDosTestCommand(),
            // new PatrolPointUnoTestCommand(),
            // new PatrolPointDosTestCommand(),
            // new PatrolPointUnoTestCommand(),
            // new PatrolPointDosTestCommand(),
            // new PatrolPointUnoTestCommand(),
            // new PatrolPointDosTestCommand(),
            // new PatrolPointUnoTestCommand(),
        );
    }
}
