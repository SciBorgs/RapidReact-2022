package frc.robot.commands.hopper;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class StartHopperCommand extends CommandBase {

    // WARNING: THIS COMMAND NEVER FINISHES BY ITSELF. USE WITH TIMEOUT OR OTHER END CONDITION.

    @Override
    public void initialize() {
        // this.addRequirements(Robot.hopperSubsystem);
    }

    @Override
    public void execute() {

        Robot.hopperSubsystem.startElevator();
        // Robot.hopperSubsystem.startSuck();
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

    
    
}
