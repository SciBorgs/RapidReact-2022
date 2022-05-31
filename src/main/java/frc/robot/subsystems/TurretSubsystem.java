package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    private final Constraints constraints = new Constraints(TurretConstants.maxV, TurretConstants.maxA);
    private final ProfiledPIDController feedback = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, constraints);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

    private double targetAngle;
    // used for calculating acceleration
    private double lastSpeed;
    private double lastTime;

    private ShuffleboardTab mainTab;

    public TurretSubsystem() {
        feedback.setTolerance(0.2);

        encoder.setDistancePerPulse(TurretConstants.DISTANCE_PER_PULSE);

        mainTab = Shuffleboard.getTab("turret  ");
        mainTab.addNumber("Current Turret Angle ", this::getCurrentAngle);
        mainTab.addNumber("Target Turret Angle", this::getTargetAngle);

        turret.setIdleMode(IdleMode.kBrake);
        
        targetAngle = 0; // (deg)

        // used for calculating acceleration
        lastSpeed = 0;
        lastTime = Timer.getFPGATimestamp();
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = MathUtil.clamp(targetAngle, -TurretConstants.LIMIT, TurretConstants.LIMIT);
    }

    public double getCurrentAngle() {
        return Units.rotationsToDegrees(encoder.getDistance() * TurretConstants.GEAR_RATIO);
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

        lastSpeed = feedback.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();

        turret.setVoltage(fb + ff);
    }
}
