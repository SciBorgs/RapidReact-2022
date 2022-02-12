package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciAbsoluteEncoder {
    private DutyCycleEncoder absEncoder;

    public SciAbsoluteEncoder(int port, double gearRatio) {
        this.absEncoder = new DutyCycleEncoder(port);
        this.absEncoder.setDistancePerRotation(gearRatio * 2 * Math.PI);
    }
    //returns the angle the hood is currently at:
    public double getAngle() {
        return absEncoder.getDistancePerRotation() * absEncoder.get();
    }
}
