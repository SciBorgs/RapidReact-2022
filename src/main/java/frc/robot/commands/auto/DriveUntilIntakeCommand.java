package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveUntilIntakeCommand extends CommandBase {
    private double speed;

    public DriveUntilIntakeCommand(double speed) {
        this.speed = speed;
    }

    public void execute() {
        Robot.driveSubsystem.setSpeed(speed, speed);
        // Robot.driveSubsystem.driveRobot(speed, speed, 0.7);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0, 0);
    }

    
    @Override
    public boolean isFinished() {
        return Robot.intakeSubsystem.getLimitSwitchState();
    }
}
