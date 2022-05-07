package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.Blockable;

@Blockable
public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax turret = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
    private final SciAbsoluteEncoder encoder = new SciAbsoluteEncoder(PortMap.TURRET_ENCODER, TurretConstants.TURRET_GEAR_RATIO, TurretConstants.OFFSET);

    private final Constraints constraints = new Constraints(TurretConstants.maxV, TurretConstants.maxA);
    private final ProfiledPIDController feedback = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, constraints);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);

    private double targetAngle;
    // used for calculating acceleration
    private double lastSpeed;
    private double lastTime;

    private ShuffleboardTab mainTab;


    public TurretSubsystem() {
        mainTab = Shuffleboard.getTab("turret  ");
        mainTab.addNumber("Current Turret Angle ", this::getCurrentAngle);
        mainTab.addNumber("Target Turret Angle", this::getTargetAngle);

        turret.setIdleMode(IdleMode.kBrake);
        
        targetAngle = 0;

        lastSpeed = 0;
        lastTime = Timer.getFPGATimestamp();
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getCurrentAngle() {
        return encoder.getAngle();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void periodic() {
        double accel = (feedback.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

        double fb = feedback.calculate(getCurrentAngle(), targetAngle);
        double ff = feedforward.calculate(feedback.getSetpoint().velocity, accel);

        lastSpeed = feedback.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();

        turret.setVoltage(fb + ff);
    }
}
