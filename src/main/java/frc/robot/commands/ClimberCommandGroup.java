package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.RunTelescopeCommand;
import frc.robot.commands.climber.RunArmCommand;

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
