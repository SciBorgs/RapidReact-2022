package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.util.PID;
import frc.robot.util.ShuffleUtil;

public class ShooterSubsystem extends SubsystemBase {
    private static double P = 0.02;
    private static PID pid = new PID(P, 0, 0);

    private CANSparkMax hood;
    //danny: It has been noted that the field, hood, w
    private ShuffleboardTab shooterTab;
    private NetworkTableEntry distance, hoodAngle;
    
    //, lmotor, rmotor;
    private SciAbsoluteEncoder thruBoreEncoder;
    public DutyCycleEncoder[] encoders = new DutyCycleEncoder[2];
    public DutyCycleEncoder hood_Encoder;

    public ShooterSubsystem() {

        this.hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        /*
        this.lmotor = new CANSparkMax(PortMap.SHOOTER_LEFT_SPARK, MotorType.kBrushless);
        this.rmotor = new CANSparkMax(PortMap.SHOOTER_RIGHT_SPARK, MotorType.kBrushless);
        */
        //lmotor.follow(rmotor);
        // hood_Encoder = hood.getAlternateEncoder(2048);
        //hood_Encoder = new DutyCycleEncoder(1);
        for(int i = 7; i < 9; i++) {
            encoders[i - 7] = new DutyCycleEncoder(i);
        }
        //relativeEncoder.setInverted(true);
        //thruBoreEncoder = new SciEncoder(relativeEncoder, Constants.TOTAL_HOOD_GEAR_RATIO, 1);
        thruBoreEncoder = new SciAbsoluteEncoder(PortMap.THRUBORE_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);

        shooterTab = Shuffleboard.getTab("Shooter");
        distance = shooterTab.add("Distance", 0.0).getEntry();
        hoodAngle = shooterTab.add("Hood Angle", 0.0).getEntry();

    }
    
    public final double HEIGHTDIFF = 9.8;
    public final double CAM_MOUNT_ANGLE = 34.523;

    public double getDistance(double limelightangle) {
        return HEIGHTDIFF/Math.tan(Math.toRadians(+CAM_MOUNT_ANGLE) + limelightangle);
    }

    public double getHoodAngle() {
        return thruBoreEncoder.getAngle();
    }

    /*
    public void shoot(double speed) {
        rmotor.set(speed);
    }
    */
    public void moveVert(double speed) {
        if(speed > 0.2)speed = -0.2;
        if(speed <-0.2) speed = 0.2;
        hood.set(speed);

    }
    public void moveHood(double angle) {

        double tmp;
        ShuffleUtil.updateEntry(hoodAngle, tmp = -pid.getOutput(angle, thruBoreEncoder.getAngle()));
        moveVert(tmp);
    }
    
    public void updateDistanceEntry(double angle) {
        ShuffleUtil.updateEntry(distance, angle);
    }
}
