package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.DummyGyro;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;
import frc.robot.Constants;

public class LocalizationSubsystem extends SubsystemBase {
    private Point pos;
    private double prevDistance, prevHeading;
    public SciEncoder totalEncoder;
    // public SciPigeon pigeon;
    public DummyGyro pigeon;
    private boolean invertedRead;

    public LocalizationSubsystem() {
        this.pos = Constants.STARTING_POINT;

        this.totalEncoder = new SciEncoder(
            Constants.WHEEL_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE,
            Robot.driveSubsystem.lFront.getEncoder(),
            Robot.driveSubsystem.lMiddle.getEncoder(),
            Robot.driveSubsystem.lBack.getEncoder(),

            Robot.driveSubsystem.rFront.getEncoder(),
            Robot.driveSubsystem.rMiddle.getEncoder(),
            Robot.driveSubsystem.rBack.getEncoder()
        );

        this.totalEncoder.setDistance(0);
        this.totalEncoder.setInverted(false, false, false, true, true, true);

        this.prevDistance = this.totalEncoder.getDistance();

        // this.pigeon = new SciPigeon(PortMap.PIGEON_ID);
        this.pigeon = new DummyGyro(
            new SciEncoder(Constants.WHEEL_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE, Robot.driveSubsystem.lFront.getEncoder()), 
            new SciEncoder(Constants.WHEEL_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE, Robot.driveSubsystem.rFront.getEncoder()));
        this.pigeon.setAngle(Constants.STARTING_HEADING);
        this.invertedRead = false;
    }

    public Point  getPos()     { return this.pos; }
    public double getX()       { return this.pos.x; }
    public double getY()       { return this.pos.y; }
    public double getVel()     { return this.totalEncoder.getSpeed(); }
    public double getHeading() { return this.invertedRead ? this.prevHeading + Math.PI : this.prevHeading; }
    public double getRawHeading() { return this.prevHeading; }

    public boolean getInverted() { return this.invertedRead; }
    public void setInverted(boolean inverted) {
        this.invertedRead = inverted;
    }

    // call in periodic
    public void update() {
        double currDistance = totalEncoder.getDistance();
        double diffDistance = currDistance - prevDistance;

        double currHeading = this.pigeon.getAngle();
        // double currHeading = 0;
        // This method of averaging angles is valid because pigeon angle is
        // continuous. If we were to normalize the angle between 0 and 2pi (or
        // some other limited range) then this wouldn't work.
        // See: https://en.wikipedia.org/wiki/Circular_mean
        double avgHeading = (currHeading + this.prevHeading) / 2;
        this.prevHeading = currHeading;

        this.pos = new Point(
            this.pos.x + diffDistance * Math.cos(avgHeading),
            this.pos.y + diffDistance * Math.sin(avgHeading));
        
        this.prevDistance = currDistance;
    }

    public void reset() {
        this.pos = Constants.STARTING_POINT;
        this.pigeon.setAngle(Constants.STARTING_HEADING);
        this.prevHeading = this.pigeon.getAngle();
    }

    public String getInfoString() {
        return "ROBOT : " + this.getPos() + " " + this.getHeading();
    }
}