package frc.robot.commands.pneumatics;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class ToggleCompressorCommand extends CommandBase {
    @Override
    public void initialize() {
        this.addRequirements(Robot.pneumaticsSubsystem);
        Robot.pneumaticsSubsystem.start();
    }
    
    @Override
    public void end(boolean interruted) { 
        Robot.pneumaticsSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
