package frc.robot.hardware;

import com.revrobotics.CANSparkMax;

public class SciSpark extends CANSparkMax {

  private boolean hasFailed = false;

  public SciSpark(int port) {
    super(port, MotorType.kBrushless);
  }

  public void set(double speed) {
    super.set(this.hasFailed ? 0 : speed);
  }

  @Override
  public void setVoltage(double outputVolts) {
    super.setVoltage(this.hasFailed ? 0 : outputVolts);
  }

  public boolean updateFailState() {
    hasFailed =
        hasFailed || this.getFault(FaultID.kMotorFault) || this.getFault(FaultID.kSensorFault);
    return hasFailed;
  }

  public void forceFailState(boolean failState) {
    this.hasFailed = failState;
  }
}
