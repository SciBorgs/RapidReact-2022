package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;
import frc.robot.Robot;

public class SpinTestCommand extends CommandBase {
    private SpinController spinController;

    private static final double HEADING_TOLERANCE = Math.PI / 6;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
    }

    @Override
    public void execute() {
        this.spinController.faceAwayFromPoint(Constants.POINT_HUB);
    }

    @Override
    public boolean isFinished() {
        return this.spinController.facingAwayFromPoint(Constants.POINT_HUB);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Stopped");
    }
}
