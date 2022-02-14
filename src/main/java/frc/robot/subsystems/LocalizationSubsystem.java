package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;

import frc.robot.Constants;

public class LocalizationSubsystem extends SubsystemBase {
    private Point pos;
    private double prevDistance, prevHeading;
    public SciEncoder totalEncoder;
    public SciPigeon pigeon;

    public LocalizationSubsystem() {
        this.pos = Constants.STARTING_POINT;

        this.totalEncoder = new SciEncoder(
            Constants.LEFT_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE,
            Robot.driveSubsystem.lFront.getEncoder(),
            // Robot.driveSubsystem.lMiddle.getEncoder(),
            Robot.driveSubsystem.lBack.getEncoder(),

            Robot.driveSubsystem.rFront.getEncoder(),
            // Robot.driveSubsystem.rMiddle.getEncoder(),
            Robot.driveSubsystem.rBack.getEncoder()
        );

        this.totalEncoder.setDistance(0);
        this.totalEncoder.setInverted(true, true, true, true);

        this.prevDistance = this.totalEncoder.getDistance();

        this.pigeon = new SciPigeon(PortMap.PIGEON_ID);
        this.pigeon.setAngle(Constants.STARTING_HEADING);
    }

    public Point  getPos()     { return this.pos; }
    public double getVel()     { return this.totalEncoder.getRate(); }
    public double getHeading() { return this.prevHeading; }

    // call in periodic
    public void update() {
        double currDistance = totalEncoder.getDistance();
        double diffDistance = currDistance - prevDistance;

        double currHeading = this.getHeading();
        double avgHeading = (currHeading + this.prevHeading) / 2;
        this.prevHeading = currHeading;

        this.pos = new Point(
            this.pos.x + diffDistance * Math.cos(avgHeading),
            this.pos.y + diffDistance * Math.sin(avgHeading));
        
        this.prevDistance = currDistance;
    }

    // resets position and angle - this does not actually move the robot
    public void reset() {
        this.pos = Constants.STARTING_POINT;
        this.pigeon.setAngle(Constants.STARTING_HEADING);
        this.totalEncoder.setDistance(0);
        this.prevDistance = this.totalEncoder.getDistance();
    }

    public String getInfoString() {
        return "POS: " + this.getPos() + " " + this.getHeading() + "\nVEL: " + this.getVel();
    }
}