package frc.robot.hardware;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class SciPigeon extends WPI_PigeonIMU {
    
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
