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

    private static final double DISTANCE_TOLERANCE = 0.3;

    @Override
    public void initialize() {
        this.axisController = new AlongAxisController(Constants.STARTING_POINT, 0, DISTANCE_TOLERANCE);
        this.axisController.setTarget(far);
        this.out = false;
    }

    @Override
    public void execute() {
        if (this.axisController.atTarget()) {
            if (out) {
                this.axisController.setTarget(close);
                // Robot.driveSubsystem.setInvertedControl(false);
                Robot.localizationSubsystem.setInverted(true);
                out = false;
            } else {
                this.axisController.setTarget(far);
                // Robot.driveSubsystem.setInvertedControl(true);
                Robot.localizationSubsystem.setInverted(false);
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
