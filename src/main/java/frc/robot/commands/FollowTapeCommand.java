package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controllers.Following;

public class FollowTapeCommand extends CommandBase {
    
    public FollowTapeCommand() {
        super();
        this.addRequirements(Robot.driveSubsystem, Robot.limelightSubsystem);
    }

    @Override
    public void execute() {
        Following.follow();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
