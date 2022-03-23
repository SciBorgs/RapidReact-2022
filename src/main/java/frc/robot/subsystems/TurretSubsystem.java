package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

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
    private static final double TX_P = 6.0 / 360;
    private PID target_pid;
    private PID reset_pid;
    private ShufflePID pidShuffleboard;

    private Averager txAverager;
    private double txAvr;
    private static final double TX_WEIGHT = 0.1;

    public TurretSubsystem() {
        this.encoder = new SciAbsoluteEncoder(PortMap.THRUBORE_ENCODER, Constants.TURRET_GEAR_RATIO);
        this.target_pid = new PID(TX_P, 0, 0);
        this.reset_pid = new PID(TX_P, 0, 0);
        this.pidShuffleboard = new ShufflePID("Turret", target_pid, "Main");
        this.txAverager = new Averager(TX_WEIGHT);
    }

    public void pointTowardsTarget(double angle) {
        txAvr = txAverager.getAverage(-angle);
        double targetAngle = encoder.getAngle() + txAvr;
        targetAngle %= LIMIT;
        double turn = target_pid.getOutput(targetAngle, encoder.getAngle());
        turn = cutToRange(turn, SPEED_LIMIT);
        motor.set(turn);
    }

    public void pointTowardsDefault() {
        double turn = reset_pid.getOutput(0, encoder.getAngle());
        turn = cutToRange(turn, SPEED_LIMIT);
        motor.set(turn);
    }

    // preventing things from going terribly wrong
    public double cutToRange(double x, double limit) {
        if (x > limit) {
            x = limit;
            System.out.println("turn > " + limit);
        } else if (x < -limit) {
            x = -limit;
            System.out.println("turn < " + limit);
        }
        return x;
    }


    public void updateShuffleboard() { 
        pidShuffleboard.update();
    }
}
