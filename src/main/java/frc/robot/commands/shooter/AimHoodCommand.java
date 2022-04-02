package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AimHoodCommand extends CommandBase {
    public static final double ANGLE = -15;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Robot.shooterSubsystem.moveHood(ANGLE);

        // for a regression, if we have time
        // double distance = Robot.shooterSubsystem.getDistance();
        // double angle = to_be_implemented(distance);
        // Robot.shooterSubsystem.moveHood(angle);
    }

    @Override
    public boolean isFinished(){
        if (Math.abs(Robot.shooterSubsystem.getCurrentHoodAngle() - ANGLE) < 0.1) {
            Robot.shooterSubsystem.stopHood();
            return true;
        }
        return false;
    }
}
