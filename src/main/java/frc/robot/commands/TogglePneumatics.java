package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class TogglePneumatics extends CommandBase {
    
    @Override
    public void initialize() {
        this.addRequirements(Robot.pneumatics);
        Robot.pneumatics.start();
    }

    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interruted) { 
        Robot.pneumatics.stop();
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
