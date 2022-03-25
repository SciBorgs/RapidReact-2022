package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShootCommand extends CommandBase {
    private static final int SPEED = 1; // speed of rotation
    private static final int LIMIT = 100; // number of rotations before the command ends

    @Override
    public void execute() {
        Robot.shooterSubsystem.run(SPEED);
    }

    @Override
    public boolean isFinished() {
        return Robot.shooterSubsystem.getDistance() < LIMIT;
    }
}
