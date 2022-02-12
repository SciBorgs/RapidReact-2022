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
    private Point posL, posR, pos;
    private double distL, distR;

    public SciEncoder leftEncoder, rightEncoder;
    public SciPigeon pigeon;

    public LocalizationSubsystem() {
        double heading = Constants.STARTING_HEADING;
        double r = Constants.ROBOT_WIDTH / 2;

        this.pos = Constants.STARTING_POINT;
        this.posL = new Point(pos.x + r * Math.cos(heading + Math.PI/2), pos.y + Math.sin(heading + Math.PI/2));
        this.posR = new Point(pos.x + r * Math.cos(heading - Math.PI/2), pos.y + Math.sin(heading - Math.PI/2));

        this.leftEncoder = new SciEncoder(
            Constants.LEFT_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE,
            Robot.driveSubsystem.lFront.getEncoder(),
            // Robot.driveSubsystem.lMiddle.getEncoder(),
            Robot.driveSubsystem.lBack.getEncoder()
        );
        this.rightEncoder  = new SciEncoder(
            Constants.LEFT_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE,
            Robot.driveSubsystem.rFront.getEncoder(),
            // Robot.driveSubsystem.rMiddle.getEncoder(),
            Robot.driveSubsystem.rBack.getEncoder()
        );

        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);

        this.leftEncoder.setInverted(true);
        this.rightEncoder.setInverted(true);

        this.distL = this.leftEncoder.getDistance();
        this.distR = this.rightEncoder.getDistance();

        this.pigeon = new SciPigeon(PortMap.PIGEON_ID);
        this.pigeon.setAngle(Constants.STARTING_HEADING);
    }

    public Point getPos() { return this.pos; }
    public double getAngle() { 
        return pigeon.getAngle(); 
    }

    // rookies can refactor this next year!!!!!!!
    // call in periodic
    public void updateLocation() {
        updateSide(this.rightEncoder, Side.RIGHT);
        updateSide(this.leftEncoder,  Side.LEFT);

        this.pos.x = (this.posL.x + this.posR.x) / 2;
        this.pos.y = (this.posL.y + this.posR.y) / 2;
    }

    private enum Side {
        RIGHT,
        LEFT
    }

    private void updateSide(SciEncoder encoder, Side side) {
        Point posS = (side == Side.RIGHT) ? this.posR   : this.posL;
        double dist  = (side == Side.RIGHT) ? this.distR : this.distL;
    
        double newDist = encoder.getDistance();
        double dDist = newDist - dist;

        double currHeading = this.getAngle();
        
        posS.x += dDist * Math.cos(currHeading);
        posS.y += dDist * Math.sin(currHeading);
        
        if (side == Side.RIGHT) this.distR += dDist; 
        else                    this.distL += dDist;
    }

    public double getVelocity() {
        return (this.leftEncoder.getRate() + this.rightEncoder.getRate()) / 2;
    }

    public void zero() {
        double heading = Constants.STARTING_HEADING;
        double r = Constants.ROBOT_WIDTH / 2;

        this.pos = new Point(0, 0);
        this.posL = new Point(0, r);
        this.posR = new Point(0, -r);

        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);

        this.distL = this.leftEncoder.get();
        this.distR = this.rightEncoder.get();

        this.pigeon.setAngle(0);
    }
}