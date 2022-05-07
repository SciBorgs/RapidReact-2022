package frc.robot.commands.climber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommandGroup extends SequentialCommandGroup {
    // TODO incomplete
    public ClimberCommandGroup(ClimberSubsystem climberSubsystem) {
        addCommands(
            new RunTelescopeCommand(false).withTimeout(2.0), 
            new RunArmCommand(true).withTimeout(2.0),
            new RunTelescopeCommand(true).withTimeout(2.0)
        );
    }
}
