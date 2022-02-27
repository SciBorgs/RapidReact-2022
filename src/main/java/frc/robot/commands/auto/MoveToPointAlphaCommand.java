package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.AlongAxisController;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveToPointAlphaCommand extends CommandBase {
    private AlongAxisController axisController;

    private static final double DISTANCE_TOLERANCE = 0.05;

    @Override
    public void initialize() {
        this.axisController = new AlongAxisController(Constants.POINT_HUB);
        this.axisController.setTargetDistance(Constants.SHOOTING_RADIUS_NEAR);
    }

    @Override
    public void execute() {
        this.axisController.move();
        Robot.localizationSubsystem.setInverted(true);
    }

    @Override
    public boolean isFinished() {
        return this.axisController.atTargetDistance(DISTANCE_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
        Robot.localizationSubsystem.setInverted(false);
    }
}
