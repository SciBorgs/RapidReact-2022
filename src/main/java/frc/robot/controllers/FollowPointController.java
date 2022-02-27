package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

public class FollowPointController {
    protected PID headingPID, distancePID;
    protected final double distanceTolerance;

    public FollowPointController(double distanceTolerance) {
        this.headingPID = new PID(5.72, 0, 0);
        this.distancePID = new PID(9.04, 0.22, 0.31);
        this.distanceTolerance = distanceTolerance;

        Robot.networkTableSubsystem.createPIDBindings("Dist PID", "dist", this.distancePID, true, true);
        Robot.networkTableSubsystem.createPIDBindings("Head PID", "head", this.headingPID, true, true);
    }

    public void move(Point targetPoint) {
        Point currPos = Robot.localizationSubsystem.getPos();
        Point displacementVector = Util.displacementVector(currPos, targetPoint);

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
