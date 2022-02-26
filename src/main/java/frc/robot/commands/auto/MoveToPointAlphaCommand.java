package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FollowPointController;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveToPointAlphaCommand extends CommandBase {
    private SpinController spinController;
    private FollowPointController pointController;

    private static final double HEADING_TOLERANCE = Math.PI / 8;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.pointController = new FollowPointController(Constants.SHOOTING_RADIUS_NEAR + 0.2);
    }

    @Override
    public void execute() {
        if (!spinController.facingPoint(Constants.POINT_HUB)) {
            this.spinController.facePoint(Constants.POINT_HUB);
        } else {
            this.pointController.move(Constants.POINT_HUB);
        }
    }

    @Override
    public boolean isFinished() {
        return pointController.hasArrived(Constants.POINT_HUB);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }
}
