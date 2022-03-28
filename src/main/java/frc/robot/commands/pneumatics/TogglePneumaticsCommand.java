package frc.robot.commands.pneumatics;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class TogglePneumaticsCommand extends CommandBase {
    @Override
    public void initialize() {
        this.addRequirements(Robot.pneumaticsSubsystem);
        Robot.pneumaticsSubsystem.start();
    }

    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interruted) { 
        Robot.pneumaticsSubsystem.stop();
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
