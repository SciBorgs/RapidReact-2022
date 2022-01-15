package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class IntakeBalls implements Command {
    
    private final double speed = 0.5;

    @Override
    public void execute() {
        Robot.intake.setSuckSpeed(speed);
    }

    public void end() {
        Robot.intake.setSuckSpeed(0);
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
