package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.PID;

public class ShooterSubsystem extends SubsystemBase {
    private static PID shooterPID = new PID(0.0005, 0, 0);
    public CANSparkMax hood;
    //, lmotor, rmotor;
    private SciAbsoluteEncoder thruBoreEncoder;
    public ShooterSubsystem() {

        this.hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
        /*
        this.lmotor = new CANSparkMax(PortMap.SHOOTER_LEFT_SPARK, MotorType.kBrushless);
        this.rmotor = new CANSparkMax(PortMap.SHOOTER_RIGHT_SPARK, MotorType.kBrushless);
        */
        
        //lmotor.follow(rmotor);
        thruBoreEncoder = new SciAbsoluteEncoder(PortMap.THRUBORE_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);
    }
    
    public final double HEIGHTDIFF = 2.08534;
    public final double CAM_MOUNT_ANGLE = 30;

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
        moveVert(-shooterPID.getOutput(angle, thruBoreEncoder.getAngle()));
    }
}
