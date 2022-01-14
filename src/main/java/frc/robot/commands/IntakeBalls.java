package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeBalls extends Command {
    
    private final double speed = 0.5;

    @Override
    public void execute() {
        Robot.intake.setSuckSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
