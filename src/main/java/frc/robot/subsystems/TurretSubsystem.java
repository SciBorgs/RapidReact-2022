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
import frc.robot.util.StateSpace;

@Blockable
public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax turret = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
    private final SciAbsoluteEncoder encoder = new SciAbsoluteEncoder(PortMap.TURRET_ENCODER, TurretConstants.GEAR_RATIO, TurretConstants.OFFSET);

    private final Constraints constraints = new Constraints(TurretConstants.maxV, TurretConstants.maxA);
    private final ProfiledPIDController feedback = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, constraints);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
    private final StateSpace turretStateSpace = new StateSpace(TurretConstants.kV, TurretConstants.kA);
    private double targetAngle;
    // used for calculating acceleration
    private double lastSpeed;
    private double lastTime;

    private ShuffleboardTab mainTab;


    public TurretSubsystem() {
        feedback.setTolerance(0.2);

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
        return feedback.atGoal();
    }
    public void setNextR(double a){
        turretStateSpace.setNextR(a);
    }

    @Override
    public void periodic() {
        

        turretStateSpace.correct(encoder.getAbsolutePosition());
        turretStateSpace.predict(0.01);

        turret.setVoltage(turretStateSpace.getU(0));
    }
}
