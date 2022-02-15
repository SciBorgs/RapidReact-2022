package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;

public class PatrolPointDosTestCommand extends CommandBase {
    private SpinController spinController;
    private MoveToPointController pointController;
    private boolean startedPIDs = false;

    private static final double HEADING_TOLERANCE = 0.2;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new MoveToPointController(Constants.POINT_PATROL_DOS);
    }

    @Override
    public void execute() {
        if (!startedPIDs) {
            this.pointController.resetPIDs();
            this.spinController.resetPIDs();
            startedPIDs = true;
        }

        if (!spinController.facingPoint(Constants.POINT_PATROL_DOS)) {
            System.out.println("SPINNING TOWARDS PATROL POINT DOS");
            this.spinController.facePoint(Constants.POINT_PATROL_DOS);
            this.pointController.resetPIDs();
        } else {
            System.out.println("APPROACHING PATROL POINT DOS");
            this.pointController.move();
            this.spinController.resetPIDs();
        }
    }

    @Override
    public boolean isFinished() {
        return this.pointController.hasArrived();
    }
}
