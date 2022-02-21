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
        double dx = targetPoint.x - currPos.x;
        double dy = targetPoint.y - currPos.y;

        double targetHeading = Math.atan2(dy, dx);
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        // We take the dot product of the displacement vector and heading
        // vector to get a kind of "signed distance". This is necessary so that
        // the robot will move backwards if it is facing away from where it 
        // should be facing
        Point displacementVector = Util.displacementVector(
            Robot.localizationSubsystem.getPos(),
            targetPoint);
        
        Point headingVector = Util.unitVector(currHeading);
        double signedDistance = Util.dot(displacementVector, headingVector);

        double angleOutput = this.headingPID.getOutput(0, diffHeading); //values negated for testing
        double forwardOutput = this.distancePID.getOutput(-signedDistance, 0);

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

    public String getInfoString() {
        return "FollowPointController : "
             + "\n\tDist  PID : " + this.distancePID.getOutput()
             + "\n\tAngle PID : " + this.headingPID.getOutput();
    }
}