package frc.robot.controllers;

import java.util.List;

import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * i implemented pure pursuit i dont even know how long ago but i'm just going to be playing around with it in the robot sim
 * (i did have to finish it up)
 * (this won't be in the actual bot, feel free to delete)
 * 
 * Adaptive Pure Pursuit (i'm just implementing normal pure pursuit, swanand sent me
 * this paper (which has a lot more than pure pursuit) for some reason)
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4.pdf
 */
public class PurePursuitController {
    private List<Point> waypoints;
    private Point next;

    private double lookahead;
    private int index;

    private PID turnPID = new PID(1, 0, 0);

    public PurePursuitController(List<Point> waypoints, double lookahead) {
        this.waypoints = waypoints;
        this.next = null;
        this.index = 0;
    }

    public List<Point> getWaypoints() { return this.waypoints; }
    public void addWaypoint(Point p) { this.waypoints.add(p); }

    /**
     * Gets the closest waypoint to the robot's current position.
     * @return the nearest waypoint
     */
    public Point closestWaypoint() {
        Point currentPos = Robot.localizationSubsystem.getPos();
        Point closestWaypoint = null;
        double closestDistanceSquared = Double.MAX_VALUE;

        int newIndex = this.index;

        for (; newIndex < this.waypoints.size(); newIndex++) {
            Point candidate = this.waypoints.get(newIndex);
            double distanceSquared = Util.distanceSquared(candidate, currentPos);

            if (distanceSquared < closestDistanceSquared) {
                closestWaypoint = candidate;
                closestDistanceSquared = distanceSquared;
            }
        }

        this.index = newIndex;

        return closestWaypoint;
    }

    /**
     * Gets the furthest waypoint from the robot's current position within the given lookahead distance.
     * @return the next waypoint
     */
    public Point nextWaypoint() {
        Point currentPos = Robot.localizationSubsystem.getPos();
        Point nextWaypoint = null;

        double lookaheadSquared = Math.pow(this.lookahead, 2);
        double distanceSquared = 0;

        for (int i = index + 1; distanceSquared <= lookaheadSquared && i < this.waypoints.size(); i++) {
            nextWaypoint = this.waypoints.get(i);
            distanceSquared = Util.distanceSquared(nextWaypoint, currentPos);
        }

        return nextWaypoint;
    }

    public void update(Point currPos, double currHeading, Point finalPos, double finalHeading) {
        this.next = this.nextWaypoint();
        double error = Util.travelledAngle(currHeading, Util.angleToPoint(Util.displacementVector(currPos, this.next)));
        turnPID.getOutput(error);
    }
}
