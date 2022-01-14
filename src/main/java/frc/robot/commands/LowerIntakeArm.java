package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


//import static frc.robot.Subsystems.intake

public class LowerIntakeArm extends Command{
    
    private final double speed = 0.9;
    
    @Override
    public void initialize(){
        setTimeout(0.1);
    }

    @Override
    public void execute(){
        Robot.intake.setArmSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return isTimedOut();
    }

    @Override
    public void end(){
        Robot.intake.setArmSpeed(0);
    }
}