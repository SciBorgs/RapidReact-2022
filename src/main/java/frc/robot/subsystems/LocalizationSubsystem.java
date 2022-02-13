package frc.robot.subsystems;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;

import frc.robot.Constants;

public class LocalizationSubsystem extends SubsystemBase {
    private Point pos;
    private double dist;
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

        this.totalEncoder.setPosition(0);
        this.totalEncoder.setInverted(true, true, true, true);

        this.dist = this.totalEncoder.getDistance();

        this.pigeon = new SciPigeon(PortMap.PIGEON_ID);
        this.pigeon.setAngle(Constants.STARTING_HEADING);
    }

    public Point  getPos()   { return this.pos; }
    public double getVel()   { return this.totalEncoder.getRate(); }
    public double getAngle() { return this.pigeon.getAngle(); }

    // call in periodic
    public void updateLocation() {
        double newDist = totalEncoder.getDistance();
        double dDist = newDist - dist;

        double currHeading = this.getAngle();
        pos.x += dDist * Math.cos(currHeading);
        pos.y += dDist * Math.sin(currHeading);
        
        this.dist = newDist;
    }

    // zeroes position and angle - this does not actually move the robot
    public void zero() {
        this.pos = new Point(0, 0);
        this.totalEncoder.setPosition(0);
        this.dist = this.totalEncoder.get();;
        this.pigeon.setAngle(0);
    }

    public String getInfoString() {
        return "POS: " + this.getPos() + " " + this.getAngle() + "\nVEL: " + this.getVel();
    }
}