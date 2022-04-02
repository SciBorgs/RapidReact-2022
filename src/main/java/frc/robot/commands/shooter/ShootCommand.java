package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShootCommand extends CommandBase {
    private static final double SPEED = 0.5; // speed of rotation
    private static final double LIMIT = 100*Constants.WHEEL_CIRCUMFERENCE; // number of rotations before the command ends

    @Override
    public void execute() {
        Robot.shooterSubsystem.runFlywheel(SPEED);
    }

    @Override
    public boolean isFinished() {
        return Robot.shooterSubsystem.getDistanceSpun() > (double)LIMIT;
    }
}