package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class IntakeBalls extends CommandBase {

    @Override
    public void initialize() {
        this.addRequirements(Robot.intake, Robot.hopper);
        Robot.intake.setSuckSpeed();
        Robot.hopper.setSuckSpeed();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stopSuck();
        Robot.hopper.stopSuck();
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
