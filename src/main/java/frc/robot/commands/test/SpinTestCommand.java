package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.SpinController;
import frc.robot.Constants;
import frc.robot.Robot;

public class SpinTestCommand extends CommandBase {
    private SpinController spinController;

    private static final double HEADING_TOLERANCE = Math.PI / 16;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.spinController.setTarget(Constants.POINT_HUB);
    }

    @Override
    public void execute() {
        this.spinController.move();
    }

    @Override
    public boolean isFinished() {
        return this.spinController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
        System.out.println("Robot Stopped");
    }
}
