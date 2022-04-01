package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunArmCommand extends CommandBase {
    boolean reversed;

    public RunArmCommand(boolean reversed) {
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        // this.addRequirements(Robot.climberSubsystem); //dont need this rn
    }

    @Override
    public void execute() {
        Robot.climberSubsystem.runArms(this.reversed);

    }

    @Override
    public void end(boolean interrupted) {
        Robot.climberSubsystem.stopArms();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
