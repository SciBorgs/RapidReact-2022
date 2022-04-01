package frc.robot.commands.climber;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunTelescopeCommand extends CommandBase {
    boolean reversed;


    public RunTelescopeCommand(boolean reversed) {
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        //this.addRequirements(Robot.climberSubsystem); //dont need this rn
    }
    
    @Override
    public void execute() {
        Robot.climberSubsystem.runTelescope(this.reversed);

    }

    @Override
    public void end(boolean interrupted) {
        Robot.climberSubsystem.stopTelescope(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    
    
}
