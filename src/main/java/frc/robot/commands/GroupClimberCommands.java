package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ExtendClimberArm;
import frc.robot.commands.climber.ExtendHook;
import frc.robot.commands.climber.RetractClimberArm;

public class GroupClimberCommands extends SequentialCommandGroup {

    public GroupClimberCommands() {
        this.addCommands(new ExtendClimberArm().withTimeout(2.0), 
                         new ExtendHook().withTimeout(2.0),
                         new RetractClimberArm().withTimeout(2.0));
        }

}
