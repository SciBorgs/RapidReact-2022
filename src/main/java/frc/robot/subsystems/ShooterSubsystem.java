package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap;
import frc.robot.sciSensors.SciAbsoluteEncoder;
import frc.robot.util.Blockable;
import frc.robot.util.StateSpace;

@Blockable
public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax hood, lmotor, rmotor;
    private final RelativeEncoder flywheelEncoder;
    private final SciAbsoluteEncoder hoodEncoder;
    
    // Hood control
    private final PIDController hoodFeedback = new PIDController(ShooterConstants.hP, ShooterConstants.hI, ShooterConstants.hD);
    private final SimpleMotorFeedforward hoodFeedforward = new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA);
    private final StateSpace hoodStateControl = new StateSpace(ShooterConstants.hV, ShooterConstants.hA);
    // Flywheel control
    private final PIDController flywheelFeedback = new PIDController(ShooterConstants.fP, ShooterConstants.fI, ShooterConstants.fD);
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.fS, ShooterConstants.fV, ShooterConstants.fA);
    private final StateSpace flyWheelStateSpace= new StateSpace(ShooterConstants.fV, ShooterConstants.fA);
    
    private double targetSpeed; // desired speed of the flywheel
    private double targetAngle; // desired angle of the hood

    private ShuffleboardTab mainTab;

    public ShooterSubsystem() {
        
        // shuffleboard
        mainTab = Shuffleboard.getTab("Shootr ");
        mainTab.addNumber("Current Hood Angle", this::getCurrentHoodAngle);
        mainTab.addNumber("Target Hood Angle", this::getTargetHoodAngle);
        mainTab.addNumber("Current Flywheel Speed", this::getCurrentFlywheelSpeed);
        mainTab.addNumber("Target Flywheel Speed", this::getTargetFlywheelSpeed);

        hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        rmotor = new CANSparkMax(PortMap.FLYWHEEL_RIGHT_SPARK, MotorType.kBrushless);
        lmotor = new CANSparkMax(PortMap.FLYWHEEL_LEFT_SPARK, MotorType.kBrushless);
        lmotor.follow(rmotor, true);

        rmotor.setIdleMode(IdleMode.kCoast);
        lmotor.setIdleMode(IdleMode.kCoast);

        rmotor.burnFlash();
        lmotor.burnFlash();

        flywheelEncoder = rmotor.getEncoder();
        flywheelEncoder.setVelocityConversionFactor(ShooterConstants.FLYWHEEL_GEAR_RATIO); // TODO add pulses if necessary?
        hoodEncoder = new SciAbsoluteEncoder(PortMap.HOOD_ENCODER, ShooterConstants.HOOD_GEAR_RATIO,
                ShooterConstants.OFFSET);

        hoodFeedback.setTolerance(0.2);
    }
    
    // FLYWHEEL
    public void setTargetFlywheelSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
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

    // HOOD ANGLE
    public void setTargetHoodAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, ShooterConstants.MAX, 0);
    }

    public double getCurrentHoodAngle() {
        return hoodEncoder.getAngle();
    }

    public double getTargetHoodAngle() {
        return targetAngle;
    }
    
    public void resetDistanceSpun() {
        flywheelEncoder.setPosition(0);
    }

    public boolean isAtTarget() {
        return hoodFeedback.atSetpoint();
    }
    public void setNextRFly(double a){
        flyWheelStateSpace.setNextR(a);
    }
    public void setNextRHood(double a){
        hoodStateControl.setNextR(a);
    }

    @Override
    public void periodic() {
       flyWheelStateSpace.correct(flywheelEncoder.getVelocity());
       flyWheelStateSpace.predict(0.02);
       rmotor.setVoltage(flyWheelStateSpace.getU(0));
        
        hoodStateControl.correct(hoodEncoder.getAbsolutePosition());
        hoodStateControl.predict(0.0001);
        hood.setVoltage(hoodStateControl.getU(0));
    }
}
