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

    // Flywheel control
    private final PIDController flywheelFeedback = new PIDController(0.9, 0, 0);

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

        rmotor.burnFlash();
        lmotor.burnFlash();

        flywheelEncoder = rmotor.getEncoder();
        flywheelFeedback.setTolerance(100);

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
        for (int i = 0; i <= 100; i++)
            System.out.println("TargetRPM: " + flywheelFeedback.atSetpoint() + "; Setpoint: " + flywheelFeedback.getSetpoint());
        return flywheelFeedback.atSetpoint();
    }

    @Override
    public void periodic() {
        // updating controllers for flywheel
        flywheelFeedback.setSetpoint(getTargetFlywheelSpeed());
        double flywheelFB = flywheelFeedback.calculate(flywheelEncoder.getVelocity(), targetSpeed);
        System.out.println("Current: " + getCurrentFlywheelSpeed() + "; Target: " + getTargetFlywheelSpeed()
                + " feedback : " + flywheelFB);

        setTargetFlywheelSpeed(flywheelFB);
        // System.out.println(flywheelFeedback.getSetpoint() + " " + getTargetFlywheelSpeed());

        if (targetSpeed == 0)
            rmotor.stopMotor();
        else
            rmotor.set(0);
        // updating ball count
        if (flywheelFeedback.getSetpoint() > 0
                && previousVelocity - flywheelEncoder.getVelocity() > ShooterConstants.DELTA_VELOCITY_THRESHOLD) {
            decrement();
        }
        previousVelocity = flywheelEncoder.getVelocity();
    }
}
 