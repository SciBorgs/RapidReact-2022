package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

public class MoveToPoint {
    private static final double EPSILON = 1E-2;
    private PID anglePid, distancePid;
    private Point targetPoint;
    private DelayedPrinter printer;

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

        Point displacementVector = Util.displacementVector(
            Robot.localizationSubsystem.getPos(),
            this.targetPoint);
        
        double currentAngle = Robot.localizationSubsystem.getAngle();

        Point headingVector = Util.unitVector(currentAngle);

        double signedDistance = Util.dot(displacementVector, headingVector);

        double angleOutput = this.anglePid.getOutput(currentAngle, targetAngle); //values negated for testing

        double forwardOutput = this.distancePid.getOutput(signedDistance, 0);

        forwardOutput = Util.normalize(forwardOutput);
        angleOutput = Util.normalize(angleOutput);

        printer.print("Distance: " + signedDistance
                    + "\nHeading: " + headingVector
                    + "\ndistance PID output: " + forwardOutput
                    + "\n\t\tangle PID output: " + angleOutput + "\n");

        Robot.driveSubsystem.setSpeedForwardAngle(forwardOutput, angleOutput);
    }

    public boolean hasArrived() {
        return Math.abs(Util.getDistance(targetPoint, Robot.localizationSubsystem.getPos())) < 0.1;
    }

}
