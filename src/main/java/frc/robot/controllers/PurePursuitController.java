package frc.robot.controllers;

import java.util.List;

import frc.robot.Robot;
import frc.robot.subsystems.NetworkTableSubsystem;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * i implemented pure pursuit i dont even know how long ago but i'm just going to be playing around with it in the robot sim
 * (i did have to finish it up)
 * (this might actually be in the robot)
 * 
 * Adaptive Pure Pursuit (i'm just implementing normal pure pursuit, swanand sent me
 * this paper (which has a lot more than pure pursuit) for some reason)
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4.pdf
 */
public class PurePursuitController implements MovementController<Point, PurePursuitController.State> {
    protected static enum State { MOVING, FINISHED }
    private State state;
    private PID turnPID;
    private Point lastPoint;

    private List<Point> waypoints;
    private Point next;
    private double length, lookahead, trackingSpeed;
    private int n, index, searchMax, endIndex;

    private static final double MAX_SEARCH_DISTANCE = 0.5;
    private static final double DISTANCE_TOLERANCE = 0.05;

    // assuming a large number of waypoints such that the path is "smooth"
    public PurePursuitController(List<Point> waypoints, double lookahead, double trackingSpeed) {
        this.waypoints = waypoints;
        this.next = waypoints.get(0);

        this.length = Util.length(waypoints, 10);
        this.lookahead = lookahead;
        this.trackingSpeed = trackingSpeed;

        this.n = waypoints.size();
        this.index = 0;
        this.searchMax = (int) (n * MAX_SEARCH_DISTANCE / this.length);
        this.endIndex  = (int) (n * (1 - DISTANCE_TOLERANCE / this.length));

        this.turnPID = new PID(2.0, 0, 0);
        this.state = State.MOVING;
        this.lastPoint = waypoints.get(n - 1);
    }

    // MovementController methods

    public Point getTargetValue() { return this.waypoints.get(this.waypoints.size() - 1); }
    public Point getCurrentValue() { return Robot.localizationSubsystem.getPos(); }
    @Deprecated public void setTarget(Point targetPoint) {}
    public boolean atTarget() { return this.index >= this.endIndex; }

    public State getCurrentState() { return this.state; }
    public boolean isFinished() {
        return this.state == State.FINISHED;
    }

    public void move() {
        if (this.atTarget()) {
            this.state = State.FINISHED;
        } else {
            this.advanceIndex();
            this.next = this.nextWaypoint();

            Point currPos = Robot.localizationSubsystem.getPos();
            double currHeading = Robot.localizationSubsystem.getHeading();
            double targetHeading = Util.angleToPoint(Util.displacementVector(currPos, this.next));

            double turnError = Util.travelledAngle(currHeading, targetHeading);
            double angle = -turnPID.getOutput(turnError);

            Robot.driveSubsystem.setSpeedForwardAngle(trackingSpeed, angle);
        }
    }

    public void stop() {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }

    // controller-specific methods

    // public List<Point> getWaypoints() { return this.waypoints; }
    // public void addWaypoint(Point p) { this.waypoints.add(p); }

    private void advanceIndex() {
        Point currentPos = Robot.localizationSubsystem.getPos();
        double closestDistanceSquared = Double.MAX_VALUE;
        int closestPointIndex = this.index;

        for (int i = this.index; i < this.index + this.searchMax && i < n; i++) {
            Point candidate = this.waypoints.get(i);
            double distanceSquared = Util.distanceSquared(candidate, currentPos);

            if (distanceSquared < closestDistanceSquared) {
                closestDistanceSquared = distanceSquared;
                closestPointIndex = i;
            }
        }

        this.index = closestPointIndex;
    }

    private Point nextWaypoint() {
        if (this.index > n) return this.lastPoint;

        Point currentPos = Robot.localizationSubsystem.getPos();
        Point nextWaypoint = null;

        double lookaheadSquared = Math.pow(this.lookahead, 2);
        double distanceSquared = 0;

        for (int i = this.index; distanceSquared <= lookaheadSquared && i < n; i++) {
            nextWaypoint = this.waypoints.get(i);
            distanceSquared = Util.distanceSquared(nextWaypoint, currentPos);
        }

        return nextWaypoint;
    }

    public void setTrackingSpeed(double speed) { this.trackingSpeed = speed; }

    public void setBindings(NetworkTableSubsystem ntsubsystem, String tab, String name) {
        ntsubsystem.createPIDBindings(tab + " pid", "pid", turnPID, true, true);
        ntsubsystem.bind(tab, "progress %", () -> 100. * this.index / this.n, 0.0);
        ntsubsystem.bind(tab, "lookahead pt", () -> this.next.toArray(), new double[] {0.0, 0.0});
        ntsubsystem.bind(tab, "tracking speed", this::setTrackingSpeed, this.trackingSpeed);
        ntsubsystem.bind(tab, "tracking index", () -> this.index, 0);
        ntsubsystem.bind(tab, "num points", () -> this.n, 0);
    }

    public void resetPIDs() {
        this.turnPID.reset();
    }
}
