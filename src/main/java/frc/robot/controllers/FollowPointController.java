package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Controls the robot so that it follows a point that may change with time.
 * Think of it like flashing a laser pointer at a point and having the robot
 * go to that point.
 */
public class FollowPointController {
    private PID headingPID, distancePID;
    private final double distanceTolerance;

    public FollowPointController(double distanceTolerance) {
        this.headingPID = new PID(5.72, 0, 0);
        this.distancePID = new PID(9.04, 0.22, 0.31);
        this.distanceTolerance = distanceTolerance;
    }

    public void move(Point targetPoint) {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = Util.displacementVector(
            currPos,
            targetPoint);

        double targetHeading = Util.angleToPoint(displacementVector);
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        double angleOutput = this.headingPID.getOutput(0, diffHeading); //values negated for testing
        double forwardOutput = this.distancePID.getOutput(Util.norm(displacementVector), 0);

        forwardOutput = Util.normalize(forwardOutput);
        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.setSpeedForwardAngle(forwardOutput, angleOutput);
    }

    public boolean hasArrived(Point targetPoint) {
        return Util.distance(targetPoint, Robot.localizationSubsystem.getPos()) < distanceTolerance;
    }

    public void resetPIDs() {
        this.headingPID.reset();
        this.distancePID.reset();
    }
}
