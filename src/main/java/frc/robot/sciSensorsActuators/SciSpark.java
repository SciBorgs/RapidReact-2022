package frc.robot.sciSensorsActuators;

import com.revrobotics.CANSparkMax;

import frc.robot.util.Util;

public class SciSpark extends CANSparkMax {

    private static double MAX_JERK = 0.15; // stalling hurt ear :(
    
    public SciSpark(int port){
        super(port, MotorType.kBrushless);
    }

    public void set(double speed) {
        double currSpeed = super.get();
        double jerk = Util.normalize(speed - currSpeed, MAX_JERK);
        super.set(currSpeed + jerk);
    }
}
