package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.subsystems.NetworkTableSubsystem;
import frc.robot.util.PID;
import frc.robot.util.Point;

import static frc.robot.util.Util.*;

public class AlongAxisController implements MovementController<Double, AlongAxisController.State> {
    protected static enum State { NONE, MOVING_NEG, MOVING_POS, FINISHED }
    private State state;
    private final PID headPID, distPID;

    private final Point origin, direction;
    private double directionAngle;
    private Point targetPoint;
    private double targetDistance;
    private double distanceTolerance;

    public AlongAxisController(Point origin, double positiveHeading, double distanceTolerance) {
        this.origin = origin;
        this.direction = unitVector(positiveHeading);
        this.directionAngle = positiveHeading;

        this.distanceTolerance = distanceTolerance;
        this.headPID = new PID(2.0, 0.3, 0.2);
        this.distPID = new PID(1.04, 0, 0);
        this.targetDistance = 0.0;
        this.targetPoint = null;
        this.state = State.NONE;
    }

    public AlongAxisController(Point origin, double distanceTolerance) {
        this(origin, angleToPoint(displacementVector(origin, Robot.localizationSubsystem.getPos())), distanceTolerance);
    }

    // MovementController methods

    public Double getTarget() { return this.targetDistance; }
    public Double getCurrentValue() { return this.distanceAlongAxis(); }
    public void setTarget(Double target) {
        this.setTargetDistance(target);
    }
    public boolean atTarget() { return this.atTargetDistance(); }

    public State getCurrentState() { return this.state; }
    public boolean isFinished() {
        return this.state == State.FINISHED;
    }

    public void move() {
        if (this.targetPoint == null) return;
        if (!this.atTarget()) {
            this.reachTargetDistance();
        } else {
            this.state = State.FINISHED;
            this.stop();
        }
    }

    public void stop() {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }

    // controller-specific methods | math

    protected double distanceAlongAxis() {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = displacementVector(origin, currPos);
        return dot(displacementVector, this.direction);
    }

    protected Point getPointAlongAxis(double distance) {
        return add(this.origin, scale(direction, distance));
    }

    protected void setTargetDistance(double targetDistance) {
        this.targetDistance = targetDistance;
        this.targetPoint = this.getPointAlongAxis(targetDistance);
    }

    protected boolean atTargetDistance() {
        return Math.abs(this.targetDistance - this.distanceAlongAxis()) < this.distanceTolerance;
    }

    protected boolean backwards() {
        return Math.abs(travelledAngle(Robot.localizationSubsystem.getHeading(), this.directionAngle)) > Math.PI / 2;
    }

    protected boolean isBehindRobot(Point p) {
        Point currPos = Robot.localizationSubsystem.getPos();
        double currHeading = Robot.localizationSubsystem.getHeading();
        Point d = displacementVector(currPos, p);
        return Math.abs(travelledAngle(angleToPoint(d), currHeading)) > Math.PI / 2;
    }

    // controller-specific methods | control

    protected void reachTargetDistance() {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = displacementVector(currPos, targetPoint);
        double targetHeading = angleToPoint(displacementVector);
        double forward = distPID.getOutput(targetDistance, distanceAlongAxis());

        if (isBehindRobot(this.targetPoint)) {
            double currHeading = Robot.localizationSubsystem.getBackwardsHeading();
            double headingError = travelledAngle(targetHeading, currHeading);
            double angle   = headPID.getOutput(0, headingError);

            this.state = State.MOVING_NEG;
            Robot.driveSubsystem.setSpeedForwardAngle(forward, angle);
        } else {
            double currHeading = Robot.localizationSubsystem.getHeading();
            double headingError = travelledAngle(targetHeading, currHeading);
            double angle   = -headPID.getOutput(0, headingError);

            this.state = State.MOVING_POS;
            Robot.driveSubsystem.setSpeedForwardAngle(forward, angle);
        }
    }

    // MovementController methods

    public void setBindings(NetworkTableSubsystem ntsubsystem) {
        ntsubsystem.createPIDBindings("axis dist PID", "dist", this.distPID, true, true);
        ntsubsystem.createPIDBindings("axis head PID", "head", this.headPID, true, true);
    }

    public void resetPIDs() {
        this.headPID.reset();
        this.distPID.reset();
    }
}
