package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

public class MoveToPoint {
    private PID anglePid, distancePid;
    private Point targetPoint;
    private DelayedPrinter printer;

    private static final double DISTANCE_TOLERANCE = 0.1;

    public MoveToPoint(Point p) {
        this.targetPoint = p;
        this.anglePid = new PID(0.7, 0, 0);
        this.distancePid = new PID(0.09, 0, 0);
        this.printer = new DelayedPrinter(100);
    }

    public void move() {
        Robot.localizationSubsystem.updateLocation();

        double dx = targetPoint.x - Robot.localizationSubsystem.getPos().x;
        double dy = targetPoint.y - Robot.localizationSubsystem.getPos().y;

        double targetAngle = Math.atan2(dy, dx);
        double currentAngle = Robot.localizationSubsystem.getAngle();

        // We take the dot product of the displacement vector and heading
        // vector to get a kind of "signed distance". This is necessary so that
        // the robot will move backwards if it is facing away from where it 
        // should be facing
        Point displacementVector = Util.displacementVector(
            Robot.localizationSubsystem.getPos(),
            this.targetPoint);
        
        Point headingVector = Util.unitVector(currentAngle);
        double signedDistance = Util.dot(displacementVector, headingVector);

        double angleOutput = this.anglePid.getOutput(currentAngle, targetAngle); //values negated for testing
        double forwardOutput = this.distancePid.getOutput(signedDistance, 0);

        forwardOutput = Util.normalize(forwardOutput);
        angleOutput = Util.normalize(angleOutput);

        printer.print("\nDistance  : " + signedDistance
                    + "\nHeading   : " + headingVector
                    + "\nDist PID  : " + forwardOutput
                    + "\nAngle PID : " + angleOutput);

        Robot.driveSubsystem.setSpeedForwardAngle(forwardOutput, angleOutput);
    }

    public boolean hasArrived() {
        return Util.getDistance(targetPoint, Robot.localizationSubsystem.getPos()) < DISTANCE_TOLERANCE;
    }
}
