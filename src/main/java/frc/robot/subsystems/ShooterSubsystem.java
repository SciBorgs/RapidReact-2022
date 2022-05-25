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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap;
import frc.robot.util.Blockable;
import frc.robot.util.BallCounter;

@Blockable
public class ShooterSubsystem extends SubsystemBase implements BallCounter {

    private final CANSparkMax hood, lmotor, rmotor;
    private final RelativeEncoder flywheelEncoder;
    private final Encoder hoodEncoder;
    
    // Hood control
    private final PIDController hoodFeedback = new PIDController(ShooterConstants.hP, ShooterConstants.hI, ShooterConstants.hD);
    private final SimpleMotorFeedforward hoodFeedforward = new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA);
    // Flywheel control
    private final PIDController flywheelFeedback = new PIDController(ShooterConstants.fP, ShooterConstants.fI, ShooterConstants.fD);
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.fS, ShooterConstants.fV, ShooterConstants.fA);
    
    private double targetSpeed; // desired speed of the flywheel (rpm)
    private double targetAngle; // desired angle of the hood (deg)

    private ShuffleboardTab mainTab;

    // keeping track of ejected balls
    private double previousVelocity;

    public ShooterSubsystem() {
        
        // shuffleboard
        mainTab = Shuffleboard.getTab("Shootr ");
        mainTab.addNumber("Current Hood Angle", this::getCurrentHoodAngle);
        mainTab.addNumber("Target Hood Angle", this::getTargetHoodAngle);
        mainTab.addNumber("Current Flywheel Speed", this::getCurrentFlywheelSpeed);
        mainTab.addNumber("Target Flywheel Speed", this::getTargetFlywheelSpeed);

        hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        hood.setInverted(true);
        rmotor = new CANSparkMax(PortMap.FLYWHEEL_RIGHT_SPARK, MotorType.kBrushless);
        lmotor = new CANSparkMax(PortMap.FLYWHEEL_LEFT_SPARK, MotorType.kBrushless);
        lmotor.follow(rmotor, true);

        rmotor.setIdleMode(IdleMode.kCoast);
        lmotor.setIdleMode(IdleMode.kCoast);

        rmotor.burnFlash();
        lmotor.burnFlash();

        flywheelEncoder = rmotor.getEncoder();
        hoodEncoder = new Encoder(PortMap.HOOD_ENCODER_A, PortMap.HOOD_ENCODER_B);
        hoodEncoder.setDistancePerPulse(ShooterConstants.DISTANCE_PER_PULSE);
        // hoodEncoder.setDistancePerRotation(ShooterConstants.HOOD_GEAR_RATIO);

        hoodFeedback.setTolerance(0.2);
        flywheelFeedback.setTolerance(0.2, 0.3); // TODO possibly update | if shooting never finishes, this is probably why

        previousVelocity = 0.0;
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
        targetAngle = MathUtil.clamp(angle, 0, ShooterConstants.MAX);
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

        // updating controllers for flywheel
        double flywheelFB = flywheelFeedback.calculate(flywheelEncoder.getVelocity(), targetSpeed);
        double flywheelFF = flywheelFeedforward.calculate(targetSpeed);
        rmotor.setVoltage(flywheelFB + flywheelFF);

        // updating ball count
        if (flywheelFeedback.getSetpoint() > 0 && previousVelocity - flywheelEncoder.getVelocity() > ShooterConstants.DELTA_VELOCITY_THRESHOLD) {
            decrement();
        }
        previousVelocity = flywheelEncoder.getVelocity();
    }
}
