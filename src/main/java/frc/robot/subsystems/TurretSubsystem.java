package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.Averager;
import frc.robot.util.PID;
import frc.robot.util.ShufflePID;

public class TurretSubsystem extends SubsystemBase {
    private CANSparkMax motor;
    private SciAbsoluteEncoder encoder;

    private double SPEED_LIMIT = 0.1;

    private final int LIMIT = 360; // change for real turret specs
    private static final double TX_P = 0.1;
    private PID pid;
    private ShufflePID pidShuffleboard;

    private Averager txAverager;
    private double txAvr;
    private static final double TX_WEIGHT = 0.1;

    public TurretSubsystem() {
        this.encoder = new SciAbsoluteEncoder(PortMap.TURRET_ENCODER, Constants.TURRET_GEAR_RATIO);
        // this.encoder.reset();
        this.pid = new PID(TX_P, 0, 0);
        this.pidShuffleboard = new ShufflePID("Turret", pid, "Main");
        this.txAverager = new Averager(TX_WEIGHT);
        this.motor = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
    }

    public void pointTowardsTarget(double angle) {
        System.out.println(encoder.getAngle());
        txAvr = txAverager.getAverage(-angle);
        double targetAngle = encoder.getAngle() + txAvr;
        targetAngle %= LIMIT;
        double turn = pid.getOutput(targetAngle, encoder.getAngle());
        turn = cutToRange(turn, SPEED_LIMIT);
        motor.set(turn);
    }

    public void stop() {
        motor.set(0);
    }

    // preventing things from going terribly wrong
    public double cutToRange(double x, double limit) {
        if (x > limit) {
            x = limit;
            System.out.println("turn > " + limit);
        } else if (x < -limit) {
            x = -limit;
            System.out.println("turn < " + -limit);
        }
        
        return x;
    }

    public double

    public void updateShuffleboard() { 
        pidShuffleboard.update();
    }
}
