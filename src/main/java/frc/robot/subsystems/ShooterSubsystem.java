package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    public CANSparkMax hood, lmotor, rmotor;

    public ShooterSubsystem() {
        this.hood = new CANSparkMax(0, MotorType.kBrushless);
        this.lmotor = new CANSparkMax(1, MotorType.kBrushless);
        this.rmotor = new CANSparkMax(2, MotorType.kBrushless);
    }
    
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
