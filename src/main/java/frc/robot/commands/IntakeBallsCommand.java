package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class IntakeBallsCommand extends CommandBase {

    @Override
    public void initialize() {
        this.addRequirements(Robot.intakeSubsystem, Robot.hopperSubsystem);
        Robot.intakeSubsystem.startSuck();
        Robot.hopperSubsystem.startSuck();
    }
 
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        Robot.intakeSubsystem.stopSuck();
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
