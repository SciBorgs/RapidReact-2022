package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap;

public class FlywheelSubsystem extends SubsystemBase {

    private final CANSparkMax motorFollow = new CANSparkMax(PortMap.Shooter.FLYWHEEL_SPARKS[0], MotorType.kBrushless); // left motor
    private final CANSparkMax motorLead = new CANSparkMax(PortMap.Shooter.FLYWHEEL_SPARKS[1], MotorType.kBrushless); // right motor
    private final RelativeEncoder flywheelEncoder = motorLead.getEncoder();
    
    // Flywheel control
    private final PIDController flywheelFeedback = new PIDController(ShooterConstants.fP, ShooterConstants.fI, ShooterConstants.fD);
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.fS, ShooterConstants.fV, ShooterConstants.fA);
    
    private double targetSpeed; // desired speed of the flywheel (rpm)

    private ShuffleboardTab tab;

    public FlywheelSubsystem() {
        // shuffleboard
        tab = Shuffleboard.getTab("Shooter");
        tab.add(this);
        tab.add("Flywheel PID", flywheelFeedback);
        tab.addNumber("Flywheel Speed", this::getCurrentFlywheelSpeed);

        // motor config
        motorFollow.follow(motorLead, true);

        motorFollow.setIdleMode(IdleMode.kCoast);
        motorLead.setIdleMode(IdleMode.kCoast);

        motorFollow.burnFlash();
        motorLead.burnFlash();

        flywheelFeedback.setTolerance(200);

        targetSpeed = 0.0;
    }
    
    // FLYWHEEL SPEED (RPM)
    public void setTargetFlywheelSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }

    // for more simple commands eg. new InstantCommand(shooter::stopFlywheel, shooter)
    public void stopFlywheel() {
        targetSpeed = 0;
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
    
    public void resetDistanceSpun() {
        flywheelEncoder.setPosition(0);
    }

    public boolean atTargetRPM() {
        return flywheelFeedback.atSetpoint();
    }
    
    @Override
    public void periodic() {        
        if (targetSpeed > 0) {
            // updating controllers for flywheel
            double fb = flywheelFeedback.calculate(flywheelEncoder.getVelocity(), targetSpeed);
            double ff = flywheelFeedforward.calculate(targetSpeed);
            motorLead.setVoltage(fb + ff);
        } else {
            motorLead.stopMotor();
        }
    }
}
