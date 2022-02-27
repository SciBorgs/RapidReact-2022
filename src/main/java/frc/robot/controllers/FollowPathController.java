package frc.robot.controllers;

import java.util.Iterator;

import frc.robot.Robot;
import frc.robot.subsystems.NetworkTableSubsystem;
import frc.robot.util.Path;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Controller for discrete path following (read "turn move turn").
 * Unlikely to be used in real auto.
 */
public class FollowPathController implements MovementController<double[], FollowPathController.State> {
    protected static enum State { NONE, SPINNING, MOVING, FINISHED };
    private State state;

    private final Iterator<Point> pathIterator;
    private SpinController spinController;
    private FollowPointController pointController;
    private Point trackingPoint;
    private boolean terminateAfterNext;

    public FollowPathController(Path path, double proceedAngle, double proceedDistance, boolean closed) {
        this.pathIterator = closed ? path.closedIterator() : path.openIterator();

        this.spinController = new SpinController(proceedAngle);
        this.pointController = new FollowPointController(proceedDistance);
        this.trackingPoint = this.pathIterator.next();

        this.terminateAfterNext = false;
        this.state = State.NONE;
    }

    // MovementController methods

    public double[] getTarget() {
        double desiredHeading = Util.angleToPoint(Util.displacementVector(Robot.localizationSubsystem.getPos(), trackingPoint));
        return new double[] { trackingPoint.x, trackingPoint.y, desiredHeading };
    }

    public double[] getCurrentValue() { 
        return Robot.localizationSubsystem.get(); 
    }

    @Deprecated public void setTarget(double[] target) {}

    public boolean atTarget() { return this.state == State.FINISHED; }

    public State getCurrentState() { return this.state; }
    public boolean isFinished() { return this.atTarget(); }

    public void move() {
        if (!this.spinController.isFinished()) {
            this.spinController.move();
            this.state = State.SPINNING;
        }
        
        else if (!this.pointController.isFinished()) {
            this.pointController.move();
            this.state = State.MOVING;
        }

        else if (!terminateAfterNext && this.pathIterator.hasNext()) {
            this.trackingPoint = this.pathIterator.next();
            this.spinController.setTarget(this.trackingPoint);
            this.pointController.setTarget(this.trackingPoint);
            this.state = State.NONE;
        }
        
        else this.state = State.FINISHED;
    }

    public void stop() {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }

    // controller-specific methods

    public void terminateAfterNext() {
        this.terminateAfterNext = true;
    }

    public Point currentPoint() {
        return this.trackingPoint;
    }

    // MovementController methods

    public void setBindings(NetworkTableSubsystem ntsubsystem) {
        this.spinController.setBindings(ntsubsystem);
        this.pointController.setBindings(ntsubsystem);
    }

    public void resetPIDs() {
        this.spinController.resetPIDs();
        this.pointController.resetPIDs();
    }
}
