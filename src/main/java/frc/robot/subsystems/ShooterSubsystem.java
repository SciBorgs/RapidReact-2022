package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap;
import frc.robot.util.Blockable;
import frc.robot.util.BallCounter;

@Blockable
public class ShooterSubsystem extends SubsystemBase implements BallCounter {

    // hardware
    private final CANSparkMax hood = new CANSparkMax(PortMap.Shooter.HOOD_SPARK, MotorType.kBrushless);
    private final CANSparkMax flywheelFollow = new CANSparkMax(PortMap.Shooter.FLYWHEEL_SPARKS[0], MotorType.kBrushless); // left motor
    private final CANSparkMax flywheelLead = new CANSparkMax(PortMap.Shooter.FLYWHEEL_SPARKS[1], MotorType.kBrushless); // right motor
    private final RelativeEncoder flywheelEncoder = flywheelLead.getEncoder();
    private final Encoder hoodEncoder = new Encoder(PortMap.Shooter.HOOD_ENCODER_QUADRATURE[0], PortMap.Shooter.HOOD_ENCODER_QUADRATURE[1]);
    
    // Hood control
    private final PIDController hoodFeedback = new PIDController(ShooterConstants.hP, ShooterConstants.hI, ShooterConstants.hD);
    private final SimpleMotorFeedforward hoodFeedforward = new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA);

    // Flywheel control
    private final PIDController flywheelFeedback = new PIDController(ShooterConstants.fP, ShooterConstants.fI, ShooterConstants.fD);
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.fS, ShooterConstants.fV, ShooterConstants.fA);
    
    private double targetSpeed; // desired speed of the flywheel (rpm)
    private double targetAngle; // desired angle of the hood (deg)
    private double lastTime; // time measurement, used for time difference in feedforward

    private ShuffleboardTab tab;

    // keeping track of ejected balls
    private double previousVelocity;

    public ShooterSubsystem() {
        
        // shuffleboard
        tab = Shuffleboard.getTab("Shooter");
        tab.add(this);
        tab.add("Flywheel PID", flywheelFeedback);
        tab.add("Hood PID", hoodFeedback);
        tab.addNumber("Current Hood Angle", this::getCurrentHoodAngle);
        tab.addNumber("Current Flywheel Speed", this::getCurrentFlywheelSpeed);

        hood.setInverted(true);
        flywheelFollow.follow(flywheelLead, true);

        flywheelFollow.setIdleMode(IdleMode.kCoast);
        flywheelLead.setIdleMode(IdleMode.kCoast);

        flywheelFollow.burnFlash();
        flywheelLead.burnFlash();

        hoodEncoder.setDistancePerPulse(ShooterConstants.DISTANCE_PER_PULSE);

        hoodFeedback.setTolerance(0.2);
        flywheelFeedback.setTolerance(200);

        targetSpeed = 0.0;
        targetAngle = 0.0;
        previousVelocity = 0.0;
        lastTime = Timer.getFPGATimestamp();
    }
    
    // FLYWHEEL SPEED (RPM)
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

    // HOOD ANGLE (DEGREES)
    public void setTargetHoodAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, 0, ShooterConstants.MAX_ANGLE);
    }

    public double getCurrentHoodAngle() {
        return Units.rotationsToDegrees(hoodEncoder.getDistance() * ShooterConstants.HOOD_GEAR_RATIO);
    }

    public double getTargetHoodAngle() {
        return targetAngle;
    }
    
    public void resetDistanceSpun() {
        flywheelEncoder.setPosition(0);
    }

    public boolean atTargetAngle() {
        return hoodFeedback.atSetpoint();
    }

    public boolean atTargetRPM() {
        return flywheelFeedback.atSetpoint();
    }
    
    @Override
    public void periodic() {

        // updating controllers for hood
        double hoodFB = hoodFeedback.calculate(getCurrentHoodAngle(), targetAngle);
        double hoodFF = hoodFeedforward.calculate(0);
        hood.setVoltage(hoodFB + hoodFF);

        
        if (targetSpeed > 0) {
            // updating controllers for flywheel
            double flywheelFB = flywheelFeedback.calculate(flywheelEncoder.getVelocity(), targetSpeed);
            double flywheelFF = flywheelFeedforward.calculate(flywheelEncoder.getVelocity(), targetSpeed, Timer.getFPGATimestamp() - lastTime);
            flywheelLead.setVoltage(flywheelFB + flywheelFF);
        } else {
            flywheelLead.stopMotor();
        }

        lastTime = Timer.getFPGATimestamp();

        // updating ball count
        if (flywheelFeedback.getSetpoint() > 0 && previousVelocity - flywheelEncoder.getVelocity() > ShooterConstants.DELTA_VELOCITY_THRESHOLD) {
            decrement();
        }
        previousVelocity = flywheelEncoder.getVelocity();
    }
}
