package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Encoder;

public class SciEncoder extends Encoder {

    private final double gearRatio; 
    private final double wheelCircumference;

    public SciEncoder(int channelA, int channelB, double gearRatio, double wheelCircumference) {
        super(channelA, channelB);
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;
    }

    public int get() {
        return (int) (super.get() * gearRatio);
    }

    public double getRate() {
        return super.getRate() * gearRatio;
    }

    // not sure if this is proper, someone fact check 
    public double getDistance() {
        return get() * wheelCircumference;
        
    }


}
