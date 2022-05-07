package frc.robot.commands.climber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommandGroup extends SequentialCommandGroup {
    // TODO incomplete
    public ClimberCommandGroup(ClimberSubsystem climber) {
        addCommands(
            new RunTelescopeCommand(climber, false).withTimeout(2.0), 
            new RunArmCommand(climber, true).withTimeout(2.0),
            new RunTelescopeCommand(climber, true).withTimeout(2.0)
        );
    }
}
