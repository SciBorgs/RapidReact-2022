package frc.robot.sciSensorsActuators;

import com.ctre.phoenix.sensors.PigeonIMU;

public class SciPigeon extends PigeonIMU {
    
    public SciPigeon(int id) {
        super(id);
    }

    public void setAngle(double angle) {
        super.setYaw(angle);
    }
}