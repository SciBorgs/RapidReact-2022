package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.AlongAxisController;
import frc.robot.util.Point;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveUpToHubCommand extends CommandBase {
    private AlongAxisController axisController;

    private static final double DISTANCE_TOLERANCE = 0.05;

    @Override
    public void initialize() {
        this.axisController = new AlongAxisController(new Point(0, 0), DISTANCE_TOLERANCE);
        this.axisController.setTarget(Constants.FENDER_RADIUS);
    }

    @Override
    public void execute() {
        this.axisController.move();
    }

    @Override
    public boolean isFinished() {
        return this.axisController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }
}
