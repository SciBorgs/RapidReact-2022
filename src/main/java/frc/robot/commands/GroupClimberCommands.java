package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GroupClimberCommands extends SequentialCommandGroup {

    public GroupClimberCommands() {
        this.addCommands(new ClimberExtend().withTimeout(2.0), 
                         new HookMotor().withTimeout(2.0));
        }

}
