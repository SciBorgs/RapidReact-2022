package frc.robot.commands.hopper;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class StartElevatorCommand extends CommandBase {
    @Override
    public void initialize() {
        this.addRequirements(Robot.hopperSubsystem);
        Robot.hopperSubsystem.startElevator();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.hopperSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    // useless functions
    @Override
    public void execute() {

    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
