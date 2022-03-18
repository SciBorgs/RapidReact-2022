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

    private final int LIMIT = 360;
    private static final double TX_P = 6.0 / 360;
    private ShufflePID pidShuffleboard;
    private PID pid;

    private Averager txAverager;
    private double txAvr;
    private static final double TX_WEIGHT = 0.1;

    private static final double DEFAULT_ANGLE = 10;

    public TurretSubsystem() {
        this.encoder = new SciAbsoluteEncoder(PortMap.THRUBORE_ENCODER, Constants.SMALL_TURRET_GEAR_RATIO);
        this.pid = new PID(TX_P, 0, 0);
        this.pidShuffleboard = new ShufflePID("Turret", pid, "Main");
        this.txAverager = new Averager(TX_WEIGHT);
    }

    // returns direction that the turret is spinning as an int, either 1 or -1
    public int getDirection() {
        if (encoder.getSpeed() > 0)
            return -1;
        return 1;
    }

    public void pointTowardsTarget(double angle) {
        txAvr = txAverager.getAverage(-angle);
        double targetAngle = getAngle() + txAvr;
        targetAngle %= LIMIT;
        double turn = pid.getOutput(targetAngle, getAngle());
        if (turn > 0.5) {
            turn = 0.5;
            System.out.println("turn > 0.5");
        } else if (turn < -0.5) {
            turn = -0.5;
            System.out.println("turn < -0.5");
        }
        turn(turn);
    }

    public void pointTowardsTarget() {
        pointTowardsTarget(getDirection() * DEFAULT_ANGLE);
    }

    public void updateShuffleboard() { 
        pidShuffleboard.update();
    }
}
