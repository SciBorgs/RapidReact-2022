package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.*;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    // public CANSparkMax lFront, lMiddle, lBack, rFront, rMiddle, rBack;
    private SciEncoder encoder;
    private final int LIMIT = 360;
    private SciPigeon pigeon;

    private static final double TX_P = 6.0 / 360;
    private ShufflePID pidShuffleboard;
    private PID pid;

    private static double avr;
    private static final double TX_WEIGHT = 0.1;

    private static final double DEFAULT_ANGLE = 10;

    public TurretSubsystem() {
       // this.lFront = new CANSparkMax(PortMap.LEFT_FRONT_SPARK, MotorType.kBrushless);
        // this.lMiddle = new CANSparkMax(PortMap.LEFT_MIDDLE_SPARK, MotorType.kBrushless);
      //  this.lBack = new CANSparkMax(PortMap.LEFT_BACK_SPARK, MotorType.kBrushless);

       // this.rFront = new CANSparkMax(PortMap.RIGHT_FRONT_SPARK, MotorType.kBrushless);
        // this.rMiddle = new CANSparkMax(PortMap.RIGHT_MIDDLE_SPARK, MotorType.kBrushless);
      //  this.rBack = new CANSparkMax(PortMap.RIGHT_BACK_SPARK, MotorType.kBrushless);

        // lMiddle.follow(lFront);
      //  lBack.follow(lFront);
        
        // rMiddle.follow(rFront);
      //  rBack.follow(rFront);

       // lFront.setIdleMode(IdleMode.kCoast);
        // lMiddle.setIdleMode(IdleMode.kCoast);
       // lBack.setIdleMode(IdleMode.kCoast);
     //   rFront.setIdleMode(IdleMode.kCoast);
       // // rMiddle.setIdleMode(IdleMode.kCoast);
      //  rBack.setIdleMode(IdleMode.kCoast);

        
        this.encoder = new SciEncoder(Constants.SMALL_TURRET_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE, lFront.getEncoder());
        pigeon = new SciPigeon(42);
        pid = new PID(TX_P, 0, 0);
        pidShuffleboard = new ShufflePID("Turret", pid, "Main");
    }

    public void setSpeed(double left, double right) {
        lFront.set(left * 0.5);
        rFront.set(-right * 0.5);
    }

    public void turn(double diff) {
        setSpeed(diff, -diff);
    }

    // should returns angle turned from the encoder
    // using pidgeon for now
    public double getAngle() {
        // return encoder.getDistance();
        return Math.toDegrees(pigeon.getAngle());
    }

    // returns direction that the turret is spinning as an int, either 1 or -1
    public int getDirection() {
        if (encoder.getSpeed() > 0)
            return -1;
        return 1;
    }

    // temporary
    public void resetPigeon() {
        pigeon.setAngle(0);
    }

    public void pointTowardsTarget(double angle) {
        avr = TX_WEIGHT * -angle + (1 - TX_WEIGHT) * avr;
        double targetAngle = getAngle() + avr;
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


//for later when we decide to use one motor for the actual turret

/*
public CANSparkMax susan;

public TurretSubsystem() {
    this.susan = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
    //Depends on how many motors there are on turret, but Im thinking there is only one for now.
    private PID pid = new PID(1, 1, 1);
}

public void turn() {
        double tx = Robot.LimeLightSubsystem.getTableData(Robot.LimeLightSubsystem.getTable(), "tx");
        double speed = pid.getOutput(0, tx);
        setTurretSpeed(speed);
}
//this might not even be needed, considering we have the followTape controller

public void setTurretSpeed(double speed) {
        this.susan.set(speed);
}
*/