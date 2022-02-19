package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class StartHopperCommand extends CommandBase {
    //Starts the hopper and elevator. Runs when held button, stops the hopper and elevator when released.

    @Override
    public void initialize() {
        this.addRequirements(Robot.hopper);
        //Robot.hopper.setElevatorSpeed();
    }

    @Override
    public void execute() {
        Robot.hopper.setSuckSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        // Robot.hopper.stopElevator();
        Robot.hopper.stopSuck();
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
