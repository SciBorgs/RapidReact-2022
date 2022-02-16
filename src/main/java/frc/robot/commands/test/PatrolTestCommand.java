package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.FollowPathController;

public class PatrolTestCommand extends CommandBase {
    private FollowPathController pathController;

    private static final double PROCEED_HEADING = Math.PI / 8;
    private static final double PROCEED_DISTANCE = 0.2;

    public PatrolTestCommand() {
        this.pathController = new FollowPathController(
            Constants.PATH_PATROL,
            PROCEED_HEADING, PROCEED_DISTANCE, true);
    }

    public void move() {
        this.pathController.move();
    }
}
