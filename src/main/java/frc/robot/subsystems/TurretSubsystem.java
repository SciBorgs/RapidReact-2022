package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.PortMap;
import frc.robot.util.Blockable;

@Blockable
public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax turret = new CANSparkMax(PortMap.Turret.TURRET_SPARK, MotorType.kBrushless);
    private final Encoder encoder = new Encoder(PortMap.Turret.TURRET_ENCODER_QUADRATURE[0], PortMap.Turret.TURRET_ENCODER_QUADRATURE[1], true);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
    // private final Constraints constraints = new Constraints(TurretConstants.maxVelocity, feedforward.maxAchievableAcceleration(TurretConstants.maxVoltage, TurretConstants.maxVelocity));
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(TurretConstants.maxVelocity, feedforward.maxAchievableAcceleration(TurretConstants.maxVoltage, TurretConstants.maxVelocity));
    private final ProfiledPIDController feedback = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, constraints);

    private double targetAngle;
    // used for calculating acceleration
    private double lastSpeed;
    private double lastTime;

    private ShuffleboardTab tab;

    public TurretSubsystem() {
        feedback.setTolerance(0.2);

        encoder.setDistancePerPulse(TurretConstants.DISTANCE_PER_PULSE * TurretConstants.GEAR_RATIO);

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
        this.targetAngle = MathUtil.clamp(targetAngle, -TurretConstants.LIMIT, TurretConstants.LIMIT);
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
        double accel = (feedback.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        // System.out.println("target " + targetAngle + "current" + getCurrentAngle());
        double fb = feedback.calculate(getCurrentAngle(), targetAngle);
        double ff = feedforward.calculate(feedback.getSetpoint().velocity, accel);
        // System.out.println("voltage = " + (fb + ff));
        // System.out.println("constraints: " + feedforward.maxAchievableVelocity(TurretConstants.maxVoltage, TurretConstants.maxAccel) + " | " + feedforward.maxAchievableAcceleration(TurretConstants.maxVoltage, TurretConstants.maxVelocity));
        // System.out.println("velocity = " + encoder.getRate());
        lastSpeed = feedback.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();

        turret.setVoltage(fb + ff);
    }
}
