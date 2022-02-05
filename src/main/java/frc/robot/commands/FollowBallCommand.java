package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controllers.Following;

public class FollowBallCommand extends CommandBase {
    public static final double LENIENCY = 1;
    public FollowBallCommand() {
        super();
        this.addRequirements(Robot.limelightSubsystem);
    }

    @Override
    public void execute() {
        Following.follow();
    }

    @Override 
    public boolean isFinished() {
        //return Robot.limelightSubsystem.getTableData(Robot.limelightSubsystem.getTable(), "tx") < LENIENCY;
        return false;
    }
}