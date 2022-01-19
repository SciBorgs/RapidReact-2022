package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class StartHopper implements Command {

    @Override
    public void initialize() {
        Robot.hopper.setSuckSpeed();
        Robot.hopper.setElevatorSpeed();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        Robot.hopper.setSuckSpeed();
        Robot.hopper.setElevatorSpeed();
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
