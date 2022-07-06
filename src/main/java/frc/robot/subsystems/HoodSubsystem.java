package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.PortMap;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends PIDSubsystem {

    private final CANSparkMax motor = new CANSparkMax(PortMap.Shooter.HOOD_SPARK, MotorType.kBrushless);
    private final Encoder encoder = new Encoder(PortMap.Shooter.HOOD_ENCODER_QUADRATURE[0], PortMap.Shooter.HOOD_ENCODER_QUADRATURE[1]);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.hS, ShooterConstants.hV, ShooterConstants.hA);

    private ShuffleboardTab tab;

    // Hood (DEG)
    public HoodSubsystem() {
        super(new PIDController(ShooterConstants.hP, ShooterConstants.hI, ShooterConstants.hD));
        getController().setTolerance(0.2);
        motor.setInverted(true);
        encoder.setDistancePerPulse(ShooterConstants.DISTANCE_PER_PULSE);

        // shuffleboard
        tab = Shuffleboard.getTab("Shooter");
        tab.add(this);
        tab.add("Hood PID", getController());
        tab.addNumber("Hood Angle", encoder::getDistance);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // ff setpoint only used for velocity, 0 here for static gain
        double ff = feedforward.calculate(0);
        motor.setVoltage(output + ff);
    }

    @Override
    protected double getMeasurement() {
        return Units.rotationsToDegrees(encoder.getDistance() * ShooterConstants.HOOD_GEAR_RATIO);
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }
}
