package frc.robot.commands.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class LowerIntakeArmCommand extends CommandBase {

    @Override
    public void initialize() {
        this.addRequirements(Robot.intakeSubsystem);
        Robot.intakeSubsystem.extendArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    // useless functions
    @Override
    public void execute() {

    }
    
}
