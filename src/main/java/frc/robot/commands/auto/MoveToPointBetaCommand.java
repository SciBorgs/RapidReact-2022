package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FollowPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveToPointBetaCommand extends CommandBase {
    private SpinController spinController;
    private FollowPointController pointController;

    private static final double HEADING_TOLERANCE = Math.PI / 4;
    private static final double DISTANCE_TOLERANCE = 0.05;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new FollowPointController(DISTANCE_TOLERANCE);

        this.spinController.setTarget(Constants.POINT_BETA);
        this.pointController.setTarget(Constants.POINT_BETA);
    }

    @Override
    public void execute() {
        if (!spinController.atTarget()) {
            this.spinController.move();
        } else {
            this.pointController.move();
        }
    }

    @Override
    public boolean isFinished() {
        return this.pointController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }
}
