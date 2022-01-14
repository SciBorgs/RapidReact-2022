package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StopIntakeBalls extends Command {

    @Override
    public void execute() {
        Robot.intake.setSuckSpeed(0);
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }
}
