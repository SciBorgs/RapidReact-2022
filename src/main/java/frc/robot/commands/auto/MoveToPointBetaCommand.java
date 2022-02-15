package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class MoveToPointBetaCommand extends CommandBase {
    private SpinController spinController;
    private MoveToPointController pointController;
    private int stage;

    private static final double HEADING_TOLERANCE = 0.3;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new MoveToPointController(Constants.POINT_BETA);
        this.stage = 1;
    }

    @Override
    public void execute() {
        if (this.stage == 1 && spinController.facingAwayFromPoint(Constants.POINT_HUB))
            this.stage++;
        
        if (stage == 1) {
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
