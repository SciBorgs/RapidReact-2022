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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sciSensors.SciAbsoluteEncoder;
import frc.robot.sciSensors.SciEncoder;
import frc.robot.PortMap;
import frc.robot.util.Blockable;
import frc.robot.util.CombinedCorrection;

@Blockable
public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax hood, lmotor, rmotor;
    private final SciEncoder flywheelEncoder;
    private final SciAbsoluteEncoder hoodEncoder;
    //NOTE TO SHOOTER PEOPLE: set constraints properly because i dont know what to put there
    CombinedCorrection<ProfiledPIDController> hoodCorrect = new CombinedCorrection<ProfiledPIDController>(new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA),
    new ProfiledPIDController(ShooterConstants.hP, ShooterConstants.hI, ShooterConstants.hD, new TrapezoidProfile.Constraints(0,0)), 0.2);

    CombinedCorrection<PIDController> flywheelCorrect = new CombinedCorrection<PIDController>(new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA), 
    new PIDController(ShooterConstants.fP, ShooterConstants.fI, ShooterConstants.fD));
    
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

        flywheelEncoder = new SciEncoder(ShooterConstants.FLYWHEEL_GEAR_RATIO, ShooterConstants.FLYWHEEL_CIRCUMFERENCE,
                rmotor.getEncoder());
        hoodEncoder = new SciAbsoluteEncoder(PortMap.HOOD_ENCODER, ShooterConstants.HOOD_GEAR_RATIO,
                ShooterConstants.OFFSET);
    }
    
    // FLYWHEEL
    public void setTargetFlywheelSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
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
        flywheelEncoder.setDistance(0);
    }

    public boolean isAtTarget() {
        return hoodCorrect.accessPID().atSetpoint();
    }

    @Override
    public void periodic() {
        rmotor.setVoltage(flywheelCorrect.getVoltage(flywheelEncoder.getSpeed(), targetSpeed));
        hood.setVoltage(hoodCorrect.getVoltage(hoodEncoder.getAngle(), targetAngle)); //idk what to set acceleration to so I just dont
    }
}
