package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Ring;
import frc.robot.util.Util;

public class MoveToRingController {
    private PID headingPID, distancePID;
    private Ring targetRing;
    private DelayedPrinter printer;

    private static final double DISTANCE_TOLERANCE = 0.1;

    public MoveToRingController(Ring targetRing, double convergenceFactor) {
        this.targetRing = targetRing;
        this.headingPID = new PID(1.0 * convergenceFactor, 0, 0);
        this.distancePID = new PID(0.1 * convergenceFactor, 0, 0);
        this.printer = new DelayedPrinter(100);
    }

    public void move() {
        Point currPos = Robot.localizationSubsystem.getPos();
        double dx = targetRing.center.x - currPos.x;
        double dy = targetRing.center.y - currPos.y;

        double targetHeading = Math.atan2(dy, dx);
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        double angleOutput = this.headingPID.getOutput(diffHeading, 0); //values negated for testing
        double forwardOutput = this.distancePID.getOutput(Util.distance(currPos, targetRing), 0);

        forwardOutput = Util.normalize(forwardOutput);
        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.setSpeedForwardAngle(forwardOutput, angleOutput);
    }

    public boolean hasArrived() {
        return Math.abs(Util.distance(Robot.localizationSubsystem.getPos(), targetRing)) < DISTANCE_TOLERANCE;
    }

    public void resetPIDs() {
        this.headingPID.reset();
        this.distancePID.reset();
    }
}
