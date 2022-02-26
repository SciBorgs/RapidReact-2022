package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controllers.FollowPathController;

public class PatrolTestCommand extends CommandBase {
    private FollowPathController pathController;
    private long startingTime;

    private static final double PROCEED_HEADING = Math.PI / 10;
    private static final double PROCEED_DISTANCE = 0.23;
    private static final long TIME_LIMIT = 75000;
    private static final long TIME_START = 5000; 

    @Override
    public void initialize() {
        this.pathController = new FollowPathController(
            Constants.PATH_PATROL,
            PROCEED_HEADING, PROCEED_DISTANCE, true);
        this.startingTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - this.startingTime > TIME_START)
            this.pathController.move();
    }

    @Override
    public boolean isFinished() {
        return Robot.isReal() && System.currentTimeMillis() - this.startingTime > TIME_LIMIT;
    }

    @Override 
    public void end(boolean interrupted) {
        if (!interrupted)
            System.out.println("PatrolTestCommand timed out. ");
    }
}
