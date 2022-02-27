package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.subsystems.NetworkTableSubsystem;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Controller for moving towards a point.
 */
public class FollowPointController implements MovementController<Point, FollowPointController.State> {
    protected static enum State { NONE, MOVING, FINISHED }
    private State state;
    private PID headingPID, distancePID;
    private Point targetPoint;
    private final double distanceTolerance;

    public FollowPointController(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        this.headingPID = new PID(5.72, 0, 0);
        this.distancePID = new PID(9.04, 0.22, 0.31);
        this.targetPoint = null;
        this.state = State.NONE;
    }

    // MovementController methods

    public Point getTarget() { return this.targetPoint; }
    public Point getCurrentValue() { return Robot.localizationSubsystem.getPos(); }
    public void setTarget(Point targetPoint) {
        this.targetPoint = targetPoint;
    }
    public boolean atTarget() { return this.hasArrived(this.targetPoint); }

    public State getCurrentState() { return this.state; }
    public boolean isFinished() {
        return this.state == State.FINISHED;
    }

    public void move() {
        if (this.targetPoint == null) return;
        if (!this.atTarget()) {
            this.state = State.MOVING;
            this.reachPoint(targetPoint);
        } else {
            this.state = State.FINISHED;
            this.stop();
        }
    }

    public void stop() {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }

    // controller-specific methods

    protected void reachPoint(Point targetPoint) {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = Util.displacementVector(currPos, targetPoint);

        double targetHeading = Util.angleToPoint(displacementVector);
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        double angleOutput = this.headingPID.getOutput(0, diffHeading); //values negated for testing
        double forwardOutput = this.distancePID.getOutput(Util.norm(displacementVector), 0);

        forwardOutput = Util.normalize(forwardOutput);
        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.setSpeedForwardAngle(forwardOutput, angleOutput);
    }

    protected boolean hasArrived(Point targetPoint) {
        return Util.distance(targetPoint, Robot.localizationSubsystem.getPos()) < distanceTolerance;
    }

    // MovementController methods

    public void setBindings(NetworkTableSubsystem ntsubsystem) {
        ntsubsystem.createPIDBindings("point dist PID", "dist", this.distancePID, true, true);
        ntsubsystem.createPIDBindings("point head PID", "head", this.headingPID, true, true);
    }

    public void resetPIDs() {
        this.headingPID.reset();
        this.distancePID.reset();
    }
}
