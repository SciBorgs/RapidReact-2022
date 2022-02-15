package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class PatrolPointUnoTestCommand extends CommandBase {
    private SpinController spinController;
    private MoveToPointController pointController;
    private int stage;

    private static final double HEADING_TOLERANCE = 0.3;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new MoveToPointController(Constants.POINT_PATROL_UNO);
        this.stage = 1;
    }

    @Override
    public void execute() {
        if (this.stage == 1 && spinController.facingPoint(Constants.POINT_PATROL_UNO))
            this.stage++;
        
        if (stage == 1) {
            this.spinController.facePoint(Constants.POINT_PATROL_UNO);
        } else {
            this.pointController.move();
        }
    }

    @Override
    public boolean isFinished() {
        return this.pointController.hasArrived();
    }
}
