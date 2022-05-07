package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.util.Blockable;

@Blockable
public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax hood, lmotor, rmotor;
    private final SciEncoder flywheelEncoder;
    private final SciAbsoluteEncoder hoodEncoder;

    private final double LOWER_LIMIT = 35.5;
    private final double UPPER_LIMIT = 9.2;
    private final double SPEED_LIMIT = 0.1;
    
    // Hood control
    private final PIDController hoodFeedback = new PIDController(ShooterConstants.hP, ShooterConstants.hI, ShooterConstants.hD);
    private final SimpleMotorFeedforward hoodFeedforward = new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA);
    // Flywheel control
    private final PIDController flywheelFeedback = new PIDController(ShooterConstants.fP, ShooterConstants.fI, ShooterConstants.fD);
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.fS, ShooterConstants.fV, ShooterConstants.fA);
    
    private double targetSpeed; // desired speed of the flywheel
    private double targetAngle; // desired angle of the hood

    private ShuffleboardTab mainTab;

    public ShooterSubsystem() {

        mainTab = Shuffleboard.getTab("Shootr ");
        mainTab.addNumber("Hood Angle", this::getHoodAngle);

        hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        rmotor = new CANSparkMax(PortMap.FLYWHEEL_RIGHT_SPARK, MotorType.kBrushless);
        lmotor = new CANSparkMax(PortMap.FLYWHEEL_LEFT_SPARK, MotorType.kBrushless);
        lmotor.follow(rmotor, true);

        rmotor.setIdleMode(IdleMode.kCoast);
        lmotor.setIdleMode(IdleMode.kCoast);

        rmotor.burnFlash();
        lmotor.burnFlash();

        flywheelEncoder = new SciEncoder(Constants.FLYWHEEL_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE, rmotor.getEncoder());
        hoodEncoder = new SciAbsoluteEncoder(PortMap.HOOD_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);
    }
    
    // FLYWHEEL
    public void setDesiredSpeed(double speed) {
        targetSpeed = speed;
    }

    public double getCurrentFlywheelSpeed() {
        return flywheelEncoder.getSpeed();
    }

    public double getTargetFlywheelSpeed() {
        return targetSpeed;
    }

    public double getDistanceSpun() {
        return flywheelEncoder.getDistance();
    }

    // HOOD ANGLE
    public void setDesiredAngle(double angle) {
        angle = translateToEncoder(angle);

        // signs are reversed because the encoder returns negative values
        if (angle < UPPER_LIMIT || angle > LOWER_LIMIT) {
            new PrintCommand("BOUNDARY");
            return;
        }
        targetAngle = angle;
    }

    public double getCurrentHoodAngle() {
        return hoodEncoder.getAngle();
    }

    public double getTargetHoodAngle() {
        return targetAngle;
    }
    
    public void resetDistanceSpun() {
        flywheelEncoder.setDistance(0);
    }

    // temp
    // public void setHoodSpeed(double speed) {
    //     System.out.println("Curr ang: " + getHoodAngle() + " speed " + speed);
    //     if (speed > 0 && getHoodAngle() > LOWER_LIMIT) {
    //         // No up
    //         System.out.println("Top boundary; cannot go up!");
    //         speed = 0;
    //     } else if (speed < 0 && getHoodAngle() < UPPER_LIMIT) {
    //         // No down
    //         System.out.println("Bottom boundary; cannot go down!");
    //         speed = 0;
    //     }
    //     hood.set(speed);
    // }

    // TODO fix hood encoder
    // normal -> crazy encoder
    private double translateToEncoder(double encoderVal) {
        return LOWER_LIMIT - encoderVal;
    }

    // crazy encoder -> normal
    public double translateFromEncoder(double val) {
        return val - LOWER_LIMIT;
    }

    @Override
    public void periodic() {
        double flywheelFB = flywheelFeedback.calculate(flywheelEncoder.getSpeed(), targetSpeed);
        double flywheelFF = flywheelFeedforward.calculate(targetSpeed);
        rmotor.setVoltage(flywheelFB + flywheelFF);
        
        double hoodFB = hoodFeedback.calculate(flywheelEncoder.getSpeed(), targetSpeed);
        double hoodFF = hoodFeedforward.calculate(targetSpeed);
        rmotor.setVoltage(hoodFB + hoodFF);
    }
}
