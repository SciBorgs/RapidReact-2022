package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.RunTelescopeCommand;
import frc.robot.commands.climber.RunArmCommand;
import frc.robot.commands.climber.RetractClimberArm;

public class GroupClimberCommands extends SequentialCommandGroup {

    public GroupClimberCommands() {
        this.addCommands(new RunTelescopeCommand().withTimeout(2.0), 
                         new RunArmCommand().withTimeout(2.0),
                         new RetractClimberArm().withTimeout(2.0));
        }

}
