package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class LowerIntakeArmCommand extends CommandBase {
    //Extends the intake arm. Runs when pressed button, instant command
    @Override
    public void initialize() {
        //this.addRequirements(Robot.intake);
        Robot.intake.extendArm();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
