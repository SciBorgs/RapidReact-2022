package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class ClimberExtend implements Command {

    @Override
    public void execute() {
        Robot.climberArm.setClimberArmSpeed();
    }

    public void end() {
        Robot.climberArm.stopClimberArm();
    }
    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
