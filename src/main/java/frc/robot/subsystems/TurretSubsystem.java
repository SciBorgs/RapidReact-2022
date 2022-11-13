package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports;

public class TurretSubsystem extends SubsystemBase {
  private final CANSparkMax turret =
      new CANSparkMax(Ports.Turret.TURRET_SPARK, MotorType.kBrushless);
  private final Encoder encoder =
      new Encoder(
          Ports.Turret.TURRET_ENCODER_QUADRATURE[0],
          Ports.Turret.TURRET_ENCODER_QUADRATURE[1],
          true);
  private final SimpleMotorFeedforward feedforward = ShooterConstants.turretFF.get();
  private final ProfiledPIDController feedback = ShooterConstants.turretProfiledPID.get();

  private double targetAngle;
  // used for calculating acceleration
  private double lastSpeed;
  private double lastTime;

  private ShuffleboardTab tab;

  public TurretSubsystem() {
    feedback.setTolerance(0.2);

    encoder.setDistancePerPulse(
        ShooterConstants.DISTANCE_PER_PULSE * ShooterConstants.TURRET_GEAR_RATIO);

    tab = Shuffleboard.getTab("Shooter");
    tab.add(this);
    tab.add("Turret Profiled PID Controller", feedback);
    tab.addNumber("Turret Angle ", this::getCurrentAngle);

    turret.setIdleMode(IdleMode.kBrake);
    // turret.setSmartCurrentLimit(1);

    turret.burnFlash();

    targetAngle = 0; // (deg)

    // used for calculating acceleration
    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle =
        MathUtil.clamp(targetAngle, -ShooterConstants.TURRET_LIMIT, ShooterConstants.TURRET_LIMIT);
  }

  public double getCurrentAngle() {
    return Units.rotationsToDegrees(encoder.getDistance());
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public boolean atTarget() {
    return feedback.atGoal();
  }

  @Override
  public void periodic() {
    double accel =
        (feedback.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    double fb = feedback.calculate(getCurrentAngle(), targetAngle);
    double ff = feedforward.calculate(feedback.getSetpoint().velocity, accel);

    lastSpeed = feedback.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();

    turret.setVoltage(fb + ff);
  }
}
