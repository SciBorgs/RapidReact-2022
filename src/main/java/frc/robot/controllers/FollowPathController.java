package frc.robot.controllers;

import java.util.Iterator;

import frc.robot.Robot;
import frc.robot.subsystems.NetworkTableSubsystem;
import frc.robot.util.Path;
import frc.robot.util.Point;

/**
 * Controller for discrete path following (read "turn move turn").
 * Unlikely to be used in real auto.
 */
public class FollowPathController {
    private final Iterator<Point> pathIterator;

    private SpinController spinController;
    private FollowPointController pointController;
    private Point trackingPoint;
    private boolean terminateAfterNext;

    private enum ControllerState { NONE, SPINNING, MOVING, FINISHED };
    private ControllerState state;

    public FollowPathController(Path path, double proceedAngle, double proceedDistance, boolean closed) {
        this.pathIterator = closed ? path.closedIterator() : path.openIterator();

        this.spinController = new SpinController(proceedAngle);
        this.pointController = new FollowPointController(proceedDistance);
        this.trackingPoint = this.pathIterator.next();

        this.terminateAfterNext = false;
        this.state = ControllerState.NONE;

        this.setBindings(Robot.networkTableSubsystem);
    }

    public void move() {
        if (!this.spinController.facingPoint(trackingPoint)) {
            this.spinController.move();
            this.state = ControllerState.SPINNING;
        }
        
        else if (!this.pointController.hasArrived(trackingPoint)) {
            this.pointController.move();
            this.state = ControllerState.MOVING;
        }

        else if (!terminateAfterNext && this.pathIterator.hasNext()) {
            this.trackingPoint = this.pathIterator.next();
            this.spinController.setTarget(this.trackingPoint);
            this.pointController.setTarget(this.trackingPoint);
            this.state = ControllerState.NONE;
        }
        
        else this.state = ControllerState.FINISHED;
    }

    public void terminateAfterNext() {
        this.terminateAfterNext = true;
    }

    public Point currentPoint() {
        return this.trackingPoint;
    }

    public boolean arrived() {
        return this.state == ControllerState.FINISHED;
    }

    public void setBindings(NetworkTableSubsystem ntsubsystem) {
        ntsubsystem.bind("FollowPathController", "state", () -> this.state.toString(), "NONE");
        ntsubsystem.bind("FollowPathController", "arrived", this::arrived, false);
        ntsubsystem.bind("FollowPathController", "termnext", () -> this.terminateAfterNext, false);
        ntsubsystem.bind("FollowPathController", "point", () -> this.currentPoint().toArray(), new double[] {0, 0});

        this.spinController.setBindings(ntsubsystem);
        this.pointController.setBindings(ntsubsystem);
    }
}
