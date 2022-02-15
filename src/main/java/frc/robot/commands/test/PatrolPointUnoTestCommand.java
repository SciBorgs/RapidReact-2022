package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class PatrolPointUnoTestCommand extends CommandBase {
    private SpinController spinController;
    private MoveToPointController pointController;
    private boolean startedPIDs = false;

    private static final double HEADING_TOLERANCE = 0.2;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new MoveToPointController(Constants.POINT_PATROL_UNO);
    }

    @Override
    public void execute() {
        if (!startedPIDs) {
            this.pointController.resetPIDs();
            this.spinController.resetPIDs();
            startedPIDs = true;
        }

        if (!spinController.facingPoint(Constants.POINT_PATROL_UNO)) {
            System.out.println("SPINNING TOWARDS PATROL POINT UNO");
            this.spinController.facePoint(Constants.POINT_PATROL_UNO);
            this.pointController.resetPIDs();
        } else {
            System.out.println("APPROACHING PATROL POINT UNO");
            this.pointController.move();
            this.spinController.resetPIDs();
        }
    }

    @Override
    public boolean isFinished() {
        return this.pointController.hasArrived();
    }
}
