package frc.robot.sciSensors;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;

public class SciSpark extends CANSparkMax {

    private static double MAX_JERK = 0.15; // stalling hurt ear :(
    private boolean hasFailed = true;

    public SciSpark(int port) {
        super(port, MotorType.kBrushless);
    }

    public void set(double speed) {
        double currSpeed = super.get();
        double jerk = MathUtil.clamp(speed - currSpeed, -MAX_JERK, MAX_JERK);
        super.set(this.hasFailed ? 0 : (currSpeed + jerk));
    }

    @Override
    public void setVoltage(double outputVolts) {
        super.setVoltage(this.hasFailed ? 0 : outputVolts);
    }

    public boolean updateFailState() {
        hasFailed = hasFailed || this.getFault(FaultID.kMotorFault) || this.getFault(FaultID.kSensorFault);
        return hasFailed;
    }

    public void forceFailState(boolean failState) {
        this.hasFailed = failState;
    }
}
