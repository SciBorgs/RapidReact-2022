package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;
import frc.robot.util.Util;
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
        this.totalEncoder.setInverted(true, true, false, false);

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

        double currHeading = this.pigeon.getAngle();
        double avgHeading = (currHeading + this.prevHeading) / 2;
        this.prevHeading = currHeading;

        this.pos = new Point(
            this.pos.x + diffDistance * Math.cos(avgHeading),
            this.pos.y + diffDistance * Math.sin(avgHeading));
        
        this.prevDistance = currDistance;
    }

    /**
     * Don't call this method.
     * <p>
     * Don't call this method, or your position will be reversed and if you
     * have any auto commands running they will see that the position of the
     * robot has moved rapidly and this will mess everything up and cause
     * the robot to go Sicko Mode and quite possibly break the robot or, more
     * likely, a student.
     * <p>
     * Don't call this method or you will be INCREDIBLY confused as to why the
     * robot goes ham when you "reset" the robot and then your robot rams into
     * the nearest wall.
     * <p>
     * PLEASE DON'T CALL THIS METHOD. I DON'T EVEN KNOW WHY IT DOESN'T WORK
     * <p>
     * AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
     * @deprecated as of Feb 14 2022
     */
    @Deprecated
    public void reset() {
        this.pos = Constants.STARTING_POINT;
        this.pigeon.setAngle(Constants.STARTING_HEADING);
        this.totalEncoder.setDistance(0);
        this.prevDistance = this.totalEncoder.getDistance();
        this.prevHeading = this.pigeon.getAngle();
        
        throw new RuntimeException("DONT CALL THIS METHOD.");
    }

    public String getInfoString() {
        return "ROBOT : " + this.getPos() + " " + this.getHeading();
    }
}