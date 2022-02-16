package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FollowPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class MoveToPointBetaCommand extends CommandBase {
    private SpinController spinController;
    private FollowPointController pointController;
    private int stage;

    private static final double HEADING_TOLERANCE = 0.3;
    private static final double DISTANCE_TOLERANCE = 0.05;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new FollowPointController(DISTANCE_TOLERANCE);
        this.stage = 1;
    }

    @Override
    public void execute() {
        if (this.stage == 1 && spinController.facingAwayFromPoint(Constants.POINT_HUB))
            this.stage++;
        
        if (stage == 1) {
            this.spinController.faceAwayFromPoint(Constants.POINT_HUB);
        } else {
            this.pointController.move(Constants.POINT_BETA);
        }
    }

    @Override
    public boolean isFinished() {
        return this.pointController.hasArrived(Constants.POINT_BETA);
    }
}
