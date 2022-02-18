package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciAbsoluteEncoder {
    private DutyCycleEncoder absEncoder;
    public double offset;

    public SciAbsoluteEncoder(int port, double gearRatio) {
        this.absEncoder = new DutyCycleEncoder(port);
        this.absEncoder.setDistancePerRotation(gearRatio * 360);
        this.offset = 0;
    }

    public void setOffset(double offset) {this.offset = offset;}
    
    //returns the angle the hood is currently at:
    public double getAngle() {return absEncoder.getDistance();}

}
