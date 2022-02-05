package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;

import frc.robot.Constants;

public class LocalizationSubsystem extends SubsystemBase {
    private Point posL, posR, pos;
    private int countL, countR;

    public Encoder leftEncoder, rightEncoder;
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
        int newCountL = this.leftEncoder.get();
        int newCountR = this.rightEncoder.get();

        int dCountL = newCountL - this.countL;
        int dCountR = newCountR - this.countR;

        double currHeading = this.getAngle();

        this.posL.x += Constants.WHEEL_CIRCUMFERENCE * dCountL * Math.cos(currHeading);
        this.posL.y += Constants.WHEEL_CIRCUMFERENCE * dCountL * Math.sin(currHeading);
        this.posR.x += Constants.WHEEL_CIRCUMFERENCE * dCountR * Math.cos(currHeading);
        this.posR.y += Constants.WHEEL_CIRCUMFERENCE * dCountR * Math.sin(currHeading);

        this.countL += dCountL;
        this.countR += dCountR;

        this.pos.x = (this.posL.x + this.posR.x) / 2;
        this.pos.y = (this.posL.y + this.posR.y) / 2;
    }
}