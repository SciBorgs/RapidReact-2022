package frc.robot.controllers;

import java.util.Iterator;

import frc.robot.util.Path;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Controls the robot so that it follows a path (closed or with endpoints).
 * No complicated math here, it's literally just spinning and moving.
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
    }

    public void move() {
        if (!this.spinController.facingPoint(trackingPoint)) {
            this.spinController.facePoint(trackingPoint);
            this.state = ControllerState.SPINNING;
        }
        
        else if (!this.pointController.hasArrived(trackingPoint)) {
            this.pointController.move(trackingPoint);
            this.state = ControllerState.MOVING;
        }

        else if (!terminateAfterNext && this.pathIterator.hasNext()) {
            this.trackingPoint = this.pathIterator.next();
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
}
