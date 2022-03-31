package frc.robot.commands.climber;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class ExtendHook extends CommandBase {
    @Override
    public void initialize() {
        //this.addRequirements(Robot.climberSubsystem); //dont need this rn
        Robot.climberSubsystem.extendHook();
    }
    
    @Override
    public void end(boolean interrupted) {
        Robot.climberSubsystem.retractHook(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
