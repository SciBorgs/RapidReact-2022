package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports;

public class HoodSubsystem extends PIDSubsystem {

  private final CANSparkMax motor = new CANSparkMax(Ports.Shooter.HOOD_SPARK, MotorType.kBrushless);
  private final Encoder encoder =
      new Encoder(
          Ports.Shooter.HOOD_ENCODER_QUADRATURE[0], Ports.Shooter.HOOD_ENCODER_QUADRATURE[1]);

  private ShuffleboardTab tab;

  // Hood (DEG)
  public HoodSubsystem() {
    super(ShooterConstants.hoodPID.get());
    getController().setTolerance(0.2);
    motor.setInverted(true);
    encoder.setDistancePerPulse(ShooterConstants.DISTANCE_PER_PULSE);

    // shuffleboard
    tab = Shuffleboard.getTab("Shooter");
    tab.add(this);
    tab.add("Hood PID", getController());
    tab.addNumber("Hood Angle", encoder::getDistance);

    enable();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    motor.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return Units.rotationsToDegrees(encoder.getDistance() * ShooterConstants.HOOD_GEAR_RATIO);
  }

  @Override
  public void setSetpoint(double setpoint) {
    setpoint = MathUtil.clamp(setpoint, 0, ShooterConstants.HOOD_LIMIT);
    super.setSetpoint(setpoint);
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }
}
