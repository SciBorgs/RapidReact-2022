package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports;

public class FlywheelSubsystem extends SubsystemBase {

  private final CANSparkMax motorLead =
      new CANSparkMax(Ports.Shooter.FLYWHEEL_RIGHT_SPARK, MotorType.kBrushless); // right motor
  private final CANSparkMax motorFollow =
      new CANSparkMax(Ports.Shooter.FLYWHEEL_LEFT_SPARK, MotorType.kBrushless); // left motor
  private final RelativeEncoder flywheelEncoder = motorLead.getEncoder();

  // Flywheel control
  private final PIDController feedback = ShooterConstants.flywheelPID.get();
  private final SimpleMotorFeedforward feedfordward = ShooterConstants.flywheelFF.get();

  private double targetSpeed; // desired speed of the flywheel (rpm)

  private ShuffleboardTab tab;

  public FlywheelSubsystem() {
    // shuffleboard
    tab = Shuffleboard.getTab("Shooter");
    tab.add(this);
    tab.add("Flywheel PID", feedback);
    tab.addNumber("Flywheel Speed", this::getCurrentFlywheelSpeed);

    // motor config
    motorLead.restoreFactoryDefaults();
    motorFollow.restoreFactoryDefaults();

    // motorLead.setInverted(true);
    motorFollow.follow(motorLead, true);

    motorLead.setIdleMode(IdleMode.kCoast);
    motorFollow.setIdleMode(IdleMode.kCoast);

    motorLead.burnFlash();
    motorFollow.burnFlash();

    // control
    feedback.setTolerance(300);

    targetSpeed = 0.0;
  }

  // FLYWHEEL SPEED (RPM)
  public void setTargetFlywheelSpeed(double targetSpeed) {
    this.targetSpeed = targetSpeed;
  }

  // for more simple commands eg. new InstantCommand(shooter::stopFlywheel, shooter)
  public void stopFlywheel() {
    targetSpeed = 0;
  }

  public double getCurrentFlywheelSpeed() {
    return flywheelEncoder.getVelocity();
  }

  public double getTargetFlywheelSpeed() {
    return targetSpeed;
  }

  public double getDistanceSpun() {
    return flywheelEncoder.getPosition();
  }

  public void resetDistanceSpun() {
    flywheelEncoder.setPosition(0);
  }

  public boolean atTargetRPM() {
    return feedback.atSetpoint();
  }

  @Override
  public void periodic() {
    // updating controllers for flywheel
    if (targetSpeed != 0) {
      double fb = feedback.calculate(flywheelEncoder.getVelocity(), targetSpeed);
      double ff = feedfordward.calculate(targetSpeed);
      motorLead.setVoltage(fb + ff);
    } else {
      motorLead.stopMotor();
    }
  }

  public CANSparkMax getSpark() {
    return motorLead;
  }
}
