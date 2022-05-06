package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.Averager;
import frc.robot.util.Blockable;
import frc.robot.util.PID;

@Blockable
public class TurretSubsystem extends pidsubsy {
    private CANSparkMax motor;
    private SciAbsoluteEncoder encoder;

    private double SPEED_LIMIT = 0.5;

    private final double LIMIT = 85; // change for real turret specs
    private static final double TX_P = 0.1;
    private PID pid;

    private ShuffleboardTab mainTab;

    private Averager txAverager;
    private double txAvr;
    private static final double TX_WEIGHT = 0.1;


    public TurretSubsystem() {
        this.encoder = new SciAbsoluteEncoder(PortMap.TURRET_ENCODER, Constants.TURRET_GEAR_RATIO);
        // this.encoder.reset();
        this.pid = new PID(TX_P, 0, 0);

        mainTab = Shuffleboard.getTab("turret  ");
        mainTab.addNumber("TURRET ANGLE Angle", this::getAngle);
        
        this.txAverager = new Averager(TX_WEIGHT);
        this.motor = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
        this.motor.setIdleMode(IdleMode.kBrake);
    
    }

    public void pointTowardsTarget(double angle) {
        System.out.println("inpAng " + angle);
        txAvr = txAverager.getAverage(-angle);
        double targetAngle = encoder.getAngle() + txAvr;
        double turn = pid.getOutput(targetAngle, encoder.getAngle());
        turn = MathUtil.clamp(turn, -SPEED_LIMIT, SPEED_LIMIT);

        if (Math.abs(targetAngle) > LIMIT) {
            turn = 0;
        }
        System.out.println("targAng " + targetAngle);

        motor.set(turn);
    }

    public void stop() {
        motor.set(0);
    }

    public double getAngle() {
        return encoder.getAngle();
    }

    public double getTarget() {
        return txAvr;
    }
}
