package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import frc.robot.util.PID;
import frc.robot.util.Point;

public class MoveToPointCommand extends CommandBase {
    private static final double EPSILON = 1E-6;
    private PID distancePid;
    private Point targetPoint;

    public MoveToPointCommand(Point targetPoint) {
        this.targetPoint = targetPoint;
        this.distancePid = new PID(1, 0, 0);
    }

    public void execute() {
        double output = this.distancePid.getOutput(targetPoint.getX(), Robot.localizationSubsystem.getX());
        Robot.driveSubsystem.setSpeed(output, output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPoint.getX() - Robot.localizationSubsystem.getX()) < EPSILON;
    }
}