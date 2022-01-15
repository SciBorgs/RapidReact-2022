package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;

public class Intake implements Subsystem {

    private CANSparkMax armSpark;
    private CANSparkMax suckSpark;

    public Intake() {
        this.armSpark = new CANSparkMax(PortMap.INTAKE_ARM_SPARK, CANSparkMax.MotorType.kBrushless);
        this.suckSpark = new CANSparkMax(PortMap.INTAKE_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
    }

    public void setArmSpeed(double speed) {
        armSpark.set(speed);
    }

    public void setSuckSpeed(double speed) {
        suckSpark.set(speed);
    }

    public void initDefaultCommand(){

    }


}