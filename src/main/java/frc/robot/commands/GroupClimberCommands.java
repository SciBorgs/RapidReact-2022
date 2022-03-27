package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ClimberExtend;
import frc.robot.commands.climber.HookMotor;
import frc.robot.commands.climber.RetractClimberArm;

public class GroupClimberCommands extends SequentialCommandGroup {

    public GroupClimberCommands() {
        this.addCommands(new ClimberExtend().withTimeout(2.0), 
                         new HookMotor().withTimeout(2.0),
                         new RetractClimberArm().withTimeout(2.0));
        }

}
