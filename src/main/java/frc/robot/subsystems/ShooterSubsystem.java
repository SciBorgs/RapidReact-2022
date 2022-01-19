package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class ShooterSubsystem extends SubsystemBase {

    public CANSparkMax hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);
    public CANSparkMax lmotor = new CANSparkMax(PortMap.SHOOTER_LEFT_SPARK, MotorType.kBrushless);
    public CANSparkMax rmotor = new CANSparkMax(PortMap.SHOOTER_RIGHT_SPARK, MotorType.kBrushless);
    
    
    public final double HEIGHTDIFF = 9.8;
    public final double CAM_MOUNT_ANGLE = 34.523;
    
    public double getDistance(double angle){
        return HEIGHTDIFF/Math.tan(Math.toRadians(+CAM_MOUNT_ANGLE));
    }
    public void shoot(double speed) {
        rmotor.set(speed);
        lmotor.follow(rmotor);
    }
    public void moveVert(double speed){
        hood.set(speed);
    }

    
}
