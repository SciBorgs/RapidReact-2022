package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.AlongAxisController;
import frc.robot.Constants;
import frc.robot.Robot;

public class AlongAxisTestCommand extends CommandBase {
    private AlongAxisController axisController;
    private double far = 5;
    private double close = -5;
    private boolean out;

    private static final double DISTANCE_TOLERANCE = 0.1;

    @Override
    public void initialize() {
        this.axisController = new AlongAxisController(Constants.POINT_HUB, 0.24);
        this.out = true;
    }

    @Override
    public void execute() {
        if (this.axisController.atTargetDistance(DISTANCE_TOLERANCE)) {
            if (out) {
                this.axisController.setTargetDistance(close);
                out = false;
            } else {
                this.axisController.setTargetDistance(far);
                out = true;
            }
        }
        this.axisController.move();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }
}
