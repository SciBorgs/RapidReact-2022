package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;


//import static frc.robot.Subsystems.intake

public class LowerIntakeArm implements Command {

    private final double speed = 0.9;

    @Override
    public void initialize(){
        withTimeout(0.1);
    }

    @Override
    public void execute(){
        Robot.intake.setArmSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(){
        Robot.intake.setArmSpeed(0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}