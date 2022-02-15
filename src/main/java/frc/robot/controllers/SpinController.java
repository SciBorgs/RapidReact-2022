package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.PID;
import frc.robot.util.Point;
import frc.robot.util.Util;

public class SpinController {
    private PID headingPID;
    private DelayedPrinter printer;
    private double headingTolerance;

    public SpinController(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        this.headingPID = new PID(0.9, 0, 0);
        this.printer = new DelayedPrinter(100);
    }

    public void reachHeading(double targetHeading) {
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, targetHeading);

        double angleOutput = this.headingPID.getOutput(diffHeading, 0); //values negated for testing

        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.spinRobot(angleOutput);
    }

    public void facePoint(Point p) {
        reachHeading(Util.angle(Util.displacementVector(Robot.localizationSubsystem.getPos(), p)));
    }

    public void faceAwayFromPoint(Point p) {
        reachHeading(Util.angle(Util.displacementVector(p, Robot.localizationSubsystem.getPos())));
    }

    public boolean facing(double heading) {
        return Math.abs(Util.travelledAngle(Robot.localizationSubsystem.getHeading(), heading)) < headingTolerance;
    }

    public boolean facingPoint(Point p) {
        return facing(Util.angle(Util.displacementVector(Robot.localizationSubsystem.getPos(), p)));
    }

    public boolean facingAwayFromPoint(Point p) {
        return facing(Util.angle(Util.displacementVector(p, Robot.localizationSubsystem.getPos())));
    }
}
