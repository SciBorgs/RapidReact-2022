package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciAbsoluteEncoder {
    private DutyCycleEncoder absEncoder;
    public double offset;
    private double factor;

    public SciAbsoluteEncoder(int port, double gearRatio) {
        this.absEncoder = new DutyCycleEncoder(port);
        factor = gearRatio * 360;
    }

    public boolean connected() {
        return this.absEncoder.isConnected();
    }

    /*
    public void setOffset(double offset) {this.offset = offset;}
    public double getOffset(double angle) {return this.offset = angle - absEncoder.getDistance();}

    public void setOffset(double offset) { absEncoder.setPositionOffset(offset); }
    public double getOffset() { return absEncoder.getPositionOffset(); }
    */

    //returns the angle the hood is currently at:
    public double getAngle() { return absEncoder.getAbsolutePosition() * factor; }
    public void reset() { absEncoder.reset(); }

}
