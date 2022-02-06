package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;

import frc.robot.Constants;

public class LocalizationSubsystem extends SubsystemBase {
    private Point posL, posR, pos;
    private int countL, countR;

    public SciEncoder leftEncoder, rightEncoder;
    public SciPigeon pigeon;

    public LocalizationSubsystem() {
        this.pigeon.setAngle(Constants.STARTING_HEADING);
        double heading = Constants.STARTING_HEADING;
        double r = Constants.ROBOT_WIDTH / 2;

        this.pos = new Point(Constants.STARTING_X, Constants.STARTING_Y);
        this.posL = new Point(pos.x + r * Math.cos(heading + Math.PI/2), pos.y + Math.sin(heading + Math.PI/2));
        this.posR = new Point(pos.x + r * Math.cos(heading - Math.PI/2), pos.y + Math.sin(heading - Math.PI/2));

        this.leftEncoder = new SciEncoder(0, 1, Constants.LEFT_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE);
        this.rightEncoder = new SciEncoder(0, 1, Constants.RIGHT_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE);
        
        this.countL = this.leftEncoder.get();
        this.countR = this.rightEncoder.get();

        this.pigeon = new SciPigeon(PortMap.PIGEON_ID);
    }

    public Point getPos() { return this.pos; }
    public double getAngle() { return pigeon.getAngle(); }

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
        int count  = (side == Side.RIGHT) ? this.countR : this.countL;
    
        int newCount = encoder.get();
        int dCount = newCount - count;

        double currHeading = this.getAngle();
        
        posS.x += Constants.WHEEL_CIRCUMFERENCE * dCount * Math.cos(currHeading);
        posS.y += Constants.WHEEL_CIRCUMFERENCE * dCount * Math.sin(currHeading);
        
        if (side == Side.RIGHT) this.countR += dCount; 
        else                    this.countL += dCount;
    }
}