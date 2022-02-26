package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Provides several methods for orienting a robot in a certain direction or
 * towards a certain point.
 */
public class SpinController {
    private PID headingPID;
    private double headingTolerance;

    public SpinController(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        this.headingPID = new PID(0.55, 0, 0);
    }

    public void reachHeading(double targetHeading) {
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        double angleOutput = this.headingPID.getOutput(diffHeading); //values negated for testing

        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.spinRobot(angleOutput);
    }

    public void faceAwayFromPoint(Point p) {
        reachHeading(Util.angleToPoint(Util.displacementVector(p, Robot.localizationSubsystem.getPos())));
    }

    public void facePoint(Point p) {
        reachHeading(Util.angleToPoint(Util.displacementVector(Robot.localizationSubsystem.getPos(), p)));
    }

    public boolean facing(double heading) {
        return Math.abs(Util.travelledAngle(Robot.localizationSubsystem.getHeading(), heading)) < headingTolerance;
    }

    public boolean facingAwayFromPoint(Point p) {
        return facing(Util.angleToPoint(Util.displacementVector(p, Robot.localizationSubsystem.getPos())));
    }

    public boolean facingPoint(Point p) {
        return facing(Util.angleToPoint(Util.displacementVector(Robot.localizationSubsystem.getPos(), p)));
    }

    public boolean facingParallelToPoint(Point p) {
        return this.facingPoint(p) || this.facingAwayFromPoint(p);
    }

    public void resetPIDs() {
        this.headingPID.reset();
    }
}
