package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Point;
import static frc.robot.util.Util.*;

public class AlongAxisController {
    private final Point origin, direction;
    private final PID headPID, distPID;

    private Point targetPoint;
    private double targetDistance;

    public AlongAxisController(Point origin, double positiveHeading) {
        this.origin = origin;
        this.direction = unitVector(positiveHeading);
        this.targetPoint = origin;
        this.targetDistance = 0.0;
        this.headPID = new PID(5.72, 0, 0);
        this.distPID = new PID(9.04, 0, 0);
    }

    public AlongAxisController(Point origin) {
        this(origin, angleToPoint(subtract(origin, Robot.localizationSubsystem.getPos())));
    }

    public double distanceAlongAxis() {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = displacementVector(origin, currPos);
        return dot(displacementVector, this.direction);
    }

    public Point getPointAlongAxis(double distance) {
        return add(this.origin, scale(direction, distance));
    }

    public void setTargetDistance(double targetDistance) {
        this.targetDistance = targetDistance;
        this.targetPoint = this.getPointAlongAxis(targetDistance);
    }

    public boolean atTargetDistance(double tolerance) {
        return Math.abs(this.targetDistance - this.distanceAlongAxis()) < tolerance;
    }

    public void move() {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = displacementVector(currPos, targetPoint);

        double targetHeading = angleToPoint(displacementVector);
        double currHeading = Robot.localizationSubsystem.getHeading();
        double headingError = travelledAngle(targetHeading, currHeading);

        double forward = distPID.getOutput(targetDistance, distanceAlongAxis());
        double angle   = headPID.getOutput(0, headingError);
        Robot.driveSubsystem.setSpeedForwardAngle(forward, angle);
    }
}
