package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class MoveToPointBetaCommand extends CommandBase {
    private SpinController spinController;
    private MoveToPointController pointController;

    private static final double HEADING_TOLERANCE = 0.2;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new MoveToPointController(Constants.POINT_BETA);
    }

    @Override
    public void execute() {
        if (!spinController.facingAwayFromPoint(Constants.POINT_HUB)) {
            this.spinController.faceAwayFromPoint(Constants.POINT_HUB);
        } else {
            this.pointController.move();
        }
    }

    @Override
    public boolean isFinished() {
        return this.pointController.hasArrived();
    }
}
