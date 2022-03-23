package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class LowerIntakeArmCommand extends CommandBase {
    @Override
    public void initialize() {
        this.addRequirements(Robot.intakeSubsystem);
        Robot.intake.startSuck();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    // useless functions
    @Override
    public void execute() {

    }
    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
