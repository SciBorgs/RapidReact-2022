package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.Averager;
import frc.robot.util.PID;
import frc.robot.util.ShufflePID;
import frc.robot.util.Util;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax motor;
    private SciAbsoluteEncoder encoder;

    private double SPEED_LIMIT = 0.5;

    private final double LIMIT = 85; // change for real turret specs
    private static final double TX_P = 0.1;
    private PID pid;
    private ShufflePID pidShuffleboard;

    private Averager txAverager;
    private double txAvr;
    private static final double TX_WEIGHT = 0.1;

    public TurretSubsystem() {
        this.encoder = new SciAbsoluteEncoder(PortMap.TURRET_ENCODER, Constants.TURRET_GEAR_RATIO);
        this.encoder.reset();
        this.pid = new PID(TX_P, 0, 0);
        this.pidShuffleboard = new ShufflePID("Turret", pid, "Main");
        this.txAverager = new Averager(TX_WEIGHT);
        this.motor = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
        this.motor.setIdleMode(IdleMode.kBrake);
    
    }

    public void pointTowardsTarget(double angle) {
        System.out.println("inpAng " + angle);
        txAvr = txAverager.getAverage(-angle);
        double targetAngle = encoder.getAngle() + txAvr;
        double turn = pid.getOutput(targetAngle, encoder.getAngle());
        
        if (Math.abs(targetAngle) > LIMIT)
            turn = 0;
        System.out.println("targAng " + targetAngle);

        turn = Util.normalize(turn, SPEED_LIMIT);
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

    public void updateShuffleboard() { 
        pidShuffleboard.update();
    }
}
