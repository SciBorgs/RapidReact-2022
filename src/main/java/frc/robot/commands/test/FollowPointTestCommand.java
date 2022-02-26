package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controllers.FollowPointController;

public class FollowPointTestCommand extends CommandBase{
    private FollowPointController controller;

    private static final double DISTANCE_TOLERANCE = 0.05;

    public FollowPointTestCommand() {
        super();
    }

    @Override
    public void initialize() {
        this.controller = new FollowPointController(DISTANCE_TOLERANCE);
    }

    @Override
    public void execute() {
        this.controller.move(Constants.POINT_HUB);
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived(Constants.POINT_HUB);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
        System.out.println("Robot Stopped");
    }
}
