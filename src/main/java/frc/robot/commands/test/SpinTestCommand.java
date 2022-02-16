package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FollowPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class SpinTestCommand extends CommandBase {
    private SpinController spinController;

    private static final double HEADING_TOLERANCE = 0.1;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
    }

    @Override
    public void execute() {
        this.spinController.facePoint(Constants.POINT_HUB);
        // this.spinController.faceAwayFromPoint(Constants.POINT_HUB);
    }

    @Override
    public boolean isFinished() {
        return this.spinController.facingPoint(Constants.POINT_HUB);
    }
}
