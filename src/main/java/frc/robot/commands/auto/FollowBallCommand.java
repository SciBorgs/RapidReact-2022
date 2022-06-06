package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controllers.Following;

public class FollowBallCommand extends CommandBase {
    public static final double LENIENCY = 1;
    public FollowBallCommand() {
        super();
        this.addRequirements(Robot.driveSubsystem, Robot.limelightSubsystem);
    }

    @Override
    public void execute() {
        System.out.println("RUNNING COMMAND!!!!!");
        Following.follow();
    }

    @Override 
    public boolean isFinished() {
        // return Robot.limelightSubsystem.getTableData(Robot.limelightSubsystem.getTable(), "tx") < LENIENCY;
        // return false;
        return Following.isFinished();
    }

    @Override
    public void end(boolean interupted) {
        Robot.driveSubsystem.setSpeed(0, 0);
    }
}
