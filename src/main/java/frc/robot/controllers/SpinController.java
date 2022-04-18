package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Controller for orienting a robot towards a certain point, in place.
 */
public class SpinController implements MovementController<Point, SpinController.State> {
    protected static enum State { NONE, SPINNING, FINISHED }
    private State state;
    private PID headingPID;
    private Point targetPoint;
    private final double headingTolerance;

    public SpinController(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        this.headingPID = new PID(0.55, 0, 0);
        this.targetPoint = null;
        this.state = State.NONE;
    }

    // MovementController methods

    public Point getTargetValue() { return this.targetPoint; }
    public Point getCurrentValue() { return Robot.localizationSubsystem.getPos(); }
    public void setTarget(Point targetPoint) {
        this.targetPoint = targetPoint;
    }
    public boolean atTarget() { return this.facingPoint(this.targetPoint); }

    public State getCurrentState() { return this.state; }
    public boolean isFinished() {
        return this.state == State.FINISHED;
    }

    public void move() {
        if (this.targetPoint == null) return;
        if (!this.atTarget()) {
            this.state = State.SPINNING;
            this.facePoint(this.targetPoint);
        } else {
            this.state = State.FINISHED;
            this.stop();
        }
    }

    public void stop() {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }

    // controller-specific methods

    protected void reachHeading(double targetHeading) {
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        double angleOutput = this.headingPID.getOutput(diffHeading); //values negated for testing

        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.spinRobot(angleOutput);
    }

    // protected void faceAwayFromPoint(Point p) {
    //     reachHeading(Util.angleToPoint(Util.displacementVector(p, Robot.localizationSubsystem.getPos())));
    // }

    protected void facePoint(Point p) {
        reachHeading(Util.angleToPoint(Util.displacementVector(Robot.localizationSubsystem.getPos(), p)));
    }

    protected boolean facing(double heading) {
        return Math.abs(Util.travelledAngle(Robot.localizationSubsystem.getHeading(), heading)) < headingTolerance;
    }

    // protected boolean facingAwayFromPoint(Point p) {
    //     return facing(Util.angleToPoint(Util.displacementVector(p, Robot.localizationSubsystem.getPos())));
    // }

    protected boolean facingPoint(Point p) {
        if (p == null) return false;
        return facing(Util.angleToPoint(Util.displacementVector(Robot.localizationSubsystem.getPos(), p)));
    }

    // protected boolean facingParallelToPoint(Point p) {
    //     return this.facingPoint(p) || this.facingAwayFromPoint(p);
    // }

    // MovementController methods

    public void setBindings(NetworkTableSubsystem ntsubsystem, String tab, String name) {
        ntsubsystem.createPIDBindings(tab + " spin pid", "pid", this.headingPID, true, true);
    }

    public void resetPIDs() {
        this.headingPID.reset();
    }
}
