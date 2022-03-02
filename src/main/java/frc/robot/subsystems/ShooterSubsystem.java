package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.PID;
import frc.robot.util.ShuffleUtil;

public class ShooterSubsystem extends SubsystemBase {
    private static PID pid = new PID(0.0005, 0, 0);
    private CANSparkMax hood;
    //danny: It has been noted that the field, hood, w
    
    private ShuffleboardTab shooterTab;
    private NetworkTableEntry distance, hoodAngle;

    //, lmotor, rmotor;
    private SciAbsoluteEncoder thruBoreEncoder;
    public ShooterSubsystem() {

        this.hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);

        // this.lmotor = new CANSparkMax(PortMap.SHOOTER_LEFT_SPARK, MotorType.kBrushless);
        // this.rmotor = new CANSparkMax(PortMap.SHOOTER_RIGHT_SPARK, MotorType.kBrushless);
        
        
        //lmotor.follow(rmotor);
        thruBoreEncoder = new SciAbsoluteEncoder(PortMap.THRUBORE_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);

        shooterTab = Shuffleboard.getTab("Shooter");
        distance = shooterTab.add("Distance", 0.0).getEntry();
        hoodAngle = shooterTab.add("Hood Angle", 0.0).getEntry();

    }
    
    public final double HEIGHTDIFF = 9.8;
    public final double CAM_MOUNT_ANGLE = 34.523;

    public double getDistance(double angle) {
        return HEIGHTDIFF/Math.tan(Math.toRadians(+CAM_MOUNT_ANGLE));
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
        if(speed > 0.1)speed = 0.1;
        if(speed <-0.1) speed = -0.1;
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
