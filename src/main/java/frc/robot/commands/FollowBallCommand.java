package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FollowBallCommand extends CommandBase {
    public static final double LENIENCY = 1;
    public FollowBallCommand() {
        super();
        this.addRequirements(Robot.driveSubsystem, Robot.limelightSubsystem);
    }

    @Override
    public void execute() {
        Robot.following.follow();
    }

    @Override 
    public boolean isFinished() {
        return Robot.limelightSubsystem.getTableData("tx") < LENIENCY;
    }
}