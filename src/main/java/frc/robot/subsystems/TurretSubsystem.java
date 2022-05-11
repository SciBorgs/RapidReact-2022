package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.PortMap;
import frc.robot.sciSensors.SciAbsoluteEncoder;
import frc.robot.util.Blockable;
import frc.robot.util.CombinedCorrection;

@Blockable
public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax turret = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
    private final SciAbsoluteEncoder encoder = new SciAbsoluteEncoder(PortMap.TURRET_ENCODER, TurretConstants.GEAR_RATIO * TurretConstants.DISTANCE_PER_PULSE, TurretConstants.OFFSET);

    CombinedCorrection<ProfiledPIDController> correction = new CombinedCorrection<ProfiledPIDController>(new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA),
    new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, new Constraints(TurretConstants.maxV, TurretConstants.maxA)), 0.2);

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
        this.targetAngle = MathUtil.clamp(targetAngle, -TurretConstants.LIMIT, TurretConstants.LIMIT);
    }

    public double getCurrentAngle() {
        return encoder.getAngle();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtTarget() {
        return correction.accessPID().atGoal();
    }

    @Override
    public void periodic() {
        double accel = (correction.accessPID().getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        turret.setVoltage(correction.getVoltage(getCurrentAngle(), targetAngle, accel));

        lastSpeed = correction.accessPID().getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }
}
