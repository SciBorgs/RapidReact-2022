package frc.robot.commands.hopper;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class StartHopperCommand extends CommandBase {
    @Override
    public void initialize() {
        this.addRequirements(Robot.hopperSubsystem);
        Robot.hopperSubsystem.startElevator();
    }

    @Override
    public void execute() {
        Robot.hopperSubsystem.startSuck();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.hopperSubsystem.stopElevator();
        Robot.hopperSubsystem.stopSuck();
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
