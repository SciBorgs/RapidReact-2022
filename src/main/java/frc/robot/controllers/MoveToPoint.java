package frc.robot.controllers;

import frc.robot.Robot;

import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

public class MoveToPoint {
    private static final double EPSILON = 1E-2;
    private PID anglePid;
    private double targetAngle;
    private PID distancePid;
    private Point targetPoint;

    public MoveToPoint(Point p) {
        double dx = p.x - Robot.localizationSubsystem.getPos().x;
        double dy = p.y - Robot.localizationSubsystem.getPos().y;
        this.targetAngle = Math.atan2(dy, dx);
        this.anglePid = new PID(0.4, 0, 0);
        this.targetPoint = p;
        this.distancePid = new PID(0.001, 0, 0);
    }

    public void turn() {
        double currentAngle = Robot.localizationSubsystem.getAngle();
        double output = this.anglePid.getOutput(targetAngle, currentAngle);
        System.out.println("turning pid: " + output);
        Robot.driveSubsystem.setSpeed(output, -output);
    }

    public boolean isFacingPoint() {
        double currentAngle = Robot.localizationSubsystem.getAngle();
        return Math.abs(targetAngle - currentAngle) < 0.1;
    }

    public void move() {
        double output = this.distancePid.getOutput(targetPoint.x, Robot.localizationSubsystem.getPos().x);
        System.out.println("moving pid: " + output);
        output = Util.normalize(output);
        Robot.driveSubsystem.setSpeed(output, output);
    }

    public boolean hasArrived() {
        return Math.abs(targetPoint.x - Robot.localizationSubsystem.getPos().x) < EPSILON;

    }

}
