package frc.robot.commands.climber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberCommandGroup extends SequentialCommandGroup {
    // TODO incomplete
    public ClimberCommandGroup() {
        addCommands(
            new RunTelescopeCommand(false).withTimeout(2.0), 
            new RunArmCommand(true).withTimeout(2.0),
            new RunTelescopeCommand(true).withTimeout(2.0)
        );
    }
}
