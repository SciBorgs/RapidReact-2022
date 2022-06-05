package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.PortMap;
import frc.robot.util.Blockable;
import frc.robot.util.SafeBangBangController;
import frc.robot.util.BallCounter;

@Blockable
public class ShooterSubsystem extends SubsystemBase implements BallCounter {

    private final CANSparkMax lmotor, rmotor;
    private final RelativeEncoder flywheelEncoder;

    private double targetSpeed; // desired speed of the flywheel (rpm)

    private ShuffleboardTab mainTab;
    private SimpleWidget targetFlyweelSpeed;

    // keeping track of ejected balls
    private double previousVelocity;

    public ShooterSubsystem() {

        rmotor = new CANSparkMax(PortMap.Shooter.FLYWHEEL_SPARKS[1], MotorType.kBrushless);
        lmotor = new CANSparkMax(PortMap.Shooter.FLYWHEEL_SPARKS[0], MotorType.kBrushless);
        lmotor.follow(rmotor, true);

        rmotor.setIdleMode(IdleMode.kCoast);
        lmotor.setIdleMode(IdleMode.kCoast);

        rmotor.setSmartCurrentLimit(30);
        lmotor.setSmartCurrentLimit(30);

        rmotor.burnFlash();
        lmotor.burnFlash();

        flywheelEncoder = rmotor.getEncoder();

        targetSpeed = 0.0;
        previousVelocity = 0.0;

        // shuffleboard
        mainTab = Shuffleboard.getTab("Shooter");
        // mainTab.addNumber("Current Flywheel Speed", this::getCurrentFlywheelSpeed);
        // mainTab.addNumber("Ball Count", this::get);

        this.targetFlyweelSpeed = mainTab.add("Target Flywheel Speed", targetSpeed);

        this.targetFlyweelSpeed.getEntry().addListener(event -> {
            this.setTargetFlywheelSpeed(event.getEntry().getDouble(this.targetSpeed));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    // FLYWHEEL
    public void setTargetFlywheelSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }

    public double getCurrentFlywheelSpeed() {
        return rmotor.get();
    }

    public double getTargetFlywheelSpeed() {
        return targetSpeed;
    }

    @Override
    public void periodic() {
        // updating controllers for flywheel
        // System.out.println("Current: " + getCurrentFlywheelSpeed() + "; Target: " + getTargetFlywheelSpeed());

        // System.out.println(flywheelFeedback.getSetpoint() + " " + getTargetFlywheelSpeed());

        rmotor.set(targetSpeed);

        System.out.println("Current RPM: " + flywheelEncoder.getVelocity());

        previousVelocity = flywheelEncoder.getVelocity();
    }
}
 