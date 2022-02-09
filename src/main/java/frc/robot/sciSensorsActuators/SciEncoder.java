package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.Encoder;

public class SciEncoder {

    private final double gearRatio; 
    private final double wheelCircumference;
    protected Encoder encoder;

    // public SciEncoder(int channelA, int channelB, double gearRatio, double wheelCircumference) {
    //     super(channelA, channelB);
    //     this.gearRatio = gearRatio;
    //     this.wheelCircumference = wheelCircumference;
    // }

    public SciEncoder(Encoder encoder, double gearRatio, double wheelCircumference) {
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;
    }

    public int get() {
        return (int) (encoder.get() * gearRatio);
    }

    public double getRate() {
        return encoder.getRate() * gearRatio;
    }

    // not sure if this is proper, someone fact check 
    public double getDistance() {
        return get() * wheelCircumference;
        
    }


}
