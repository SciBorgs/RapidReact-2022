package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RaiseHoodCommand extends CommandBase {
    public static final double SPEED = 0.1;

    @Override
    public void execute() {
        Robot.shooterSubsystem.setHoodSpeed(SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooterSubsystem.setHoodSpeed(0); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
