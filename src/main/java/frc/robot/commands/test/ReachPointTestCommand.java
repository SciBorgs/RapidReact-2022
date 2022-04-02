package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.Robot;
import frc.robot.controllers.ReachPointController;

public class ReachPointTestCommand extends CommandBase {
    private ReachPointController controller;

    private static final double DISTANCE_TOLERANCE = 0.05;

    @Override
    public void initialize() {
        this.controller = new ReachPointController(DISTANCE_TOLERANCE);
        this.controller.setTarget(AutoProfile.POINT_TEST);
    }

    @Override
    public void execute() {
        this.controller.move();
    }

    @Override
    public boolean isFinished() {
        return controller.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
        System.out.println("Robot Stopped");
    }
}
