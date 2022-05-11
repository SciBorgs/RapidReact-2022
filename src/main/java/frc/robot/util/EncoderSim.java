package frc.robot.util;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class EncoderSim {
    private final SimDouble position, velocity;
    
    public EncoderSim(int deviceID) {
        SimDeviceSim device = new SimDeviceSim("SPARK MAX [" + deviceID + "]");
        position = device.getDouble("Position");
        velocity = device.getDouble("Velocity");
    }
    
    public void setVelocity(double v) {
        velocity.set(v);
    }

    public void setPosition(double pos) {
        position.set(pos);
    }

}
