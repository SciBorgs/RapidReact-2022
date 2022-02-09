package frc.robot.controllers;
import frc.robot.Robot;

import frc.robot.util.PID;
import frc.robot.util.Point;

public class MoveToPoint {
    private static final double EPSILON = 1E-6;
    private PID anglePid;
    private double targetAngle;
    private PID distancePid;
    private Point targetPoint;

    public MoveToPoint(Point p) {
        double dx = p.x - Robot.localizationSubsystem.getPos().x;
        double dy = p.y - Robot.localizationSubsystem.getPos().y;
        this.targetAngle = Math.atan2(dy, dx);
        this.anglePid = new PID(0.00, 0, 0);
        this.targetPoint = p;
        this.distancePid = new PID(0.005, 0, 0);
    }

    public void turn() {
        double currentAngle = Robot.localizationSubsystem.getAngle();
        double output = this.anglePid.getOutput(targetAngle, currentAngle);
        Robot.driveSubsystem.setSpeedForwardAngle(0.1, output);
    }

    public boolean isFacingPoint() {
        double currentAngle = Robot.localizationSubsystem.getAngle();
        return Math.abs(targetAngle - currentAngle) < EPSILON;
    }

    public void move() {
        double output = this.distancePid.getOutput(targetPoint.x, Robot.localizationSubsystem.getPos().x);
        Robot.driveSubsystem.setSpeed(output, output);
    }

    public boolean hasArrived() {
        return Math.abs(targetPoint.x - Robot.localizationSubsystem.getPos().x) < EPSILON;

    }

}
