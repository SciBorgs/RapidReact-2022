package frc.robot.subsystems;

//import frc.robot.Utils;
//import frc.robot.PortMap;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;

public class ClimberSubsystem extends Subsystem {

    private CANSparkMax fixedArm, secondArmMover, secondArm;
    private final double FIXED_ARM_SPEED = 0.5;

    //private somethingforcamera shadowLineCamera;

    public ClimberSubsystem() {
        
        this.fixedArm = new CANSparkMax();

        this.secondArmMover = new CANSparkMax();
        this.secondArm = new CANSparkMax();

        //this.shadowLineCamera = new somethingforcamera();
    
    }

    public void extendFixedArm() {
        this.fixedArm.set(this.FIXED_ARM_SPEED);
    
    }



    public void extendSecondArm() {

    }

    
}
