package frc.robot.sciSensors;

import com.ctre.phoenix.sensors.PigeonIMU;

public class SciPigeon extends PigeonIMU {
    
    public SciPigeon(int id) {
        super(id);
    }

    private double[] yawPitchRoll() {
        double[] yawPitchRollValues = new double[3];
        super.getYawPitchRoll(yawPitchRollValues);
        return yawPitchRollValues;
    }

    public double getAngle() { return Math.toRadians(yawPitchRoll()[0]); }
    public double getPitch() { return Math.toRadians(yawPitchRoll()[1]); }
    public double getRoll() { return Math.toRadians(yawPitchRoll()[2]); }

    public void setAngle(double angle) {
        super.setYaw(Math.toDegrees(angle));
    }
}
