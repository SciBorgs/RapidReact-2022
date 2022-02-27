package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.subsystems.NetworkTableSubsystem;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

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
        this.headPID = new PID(2.0, 0.3, 0.2);
        this.distPID = new PID(1.04, 0, 0);

        NetworkTableSubsystem binder = Robot.networkTableSubsystem;
        binder.bind("axis test", "target dist",   () -> this.targetDistance, 0.1156);
        binder.bind("axis test", "target point",  () -> this.targetPoint.toArray(), new double[] {0.0, 0.0});
        binder.bind("axis test", "init origin", () -> this.origin.toArray(), new double[] {0.0, 0.0});
        binder.bind("axis test", "init direct", () -> Util.angleToPoint(this.direction), 0.1156);
        binder.bind("axis test", "run finished", () -> this.atTargetDistance(), true);
        binder.bind("axis test", "run distance", this::distanceAlongAxis, 0.0);
        binder.createPIDBindings("axis test [head pid]", "pid", this.headPID, true, true);
        binder.createPIDBindings("axis test [dist pid]", "pid", this.distPID, true, true);
    }

    public AlongAxisController(Point origin) {
        this(origin, angleToPoint(displacementVector(origin, Robot.localizationSubsystem.getPos())));
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

    public boolean atTargetDistance() {
        return this.atTargetDistance(5.0E-2);
    }

    public void move() {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = displacementVector(currPos, targetPoint);

        double targetHeading = angleToPoint(displacementVector);
        double currHeading = Robot.localizationSubsystem.getHeading();
        double headingError = travelledAngle(targetHeading, currHeading);

        double forward = distPID.getOutput(targetDistance, distanceAlongAxis());
        double angle   = -headPID.getOutput(0, headingError);
        if (Robot.localizationSubsystem.getInverted()) angle *= -1.0;
        Robot.driveSubsystem.setSpeedForwardAngle(forward, angle);
    }
}
