package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;

public class Intake implements Subsystem {

    private DoubleSolenoid armSolenoid;
    private CANSparkMax suckSpark;

    private final double INTAKE_SPEED = 0.5;

    public Intake() {
        this.armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortMap.INTAKE_ARM_FORWARD_CHANNEL, PortMap.INTAKE_ARM_REVERSE_CHANNEL);
        this.suckSpark = new CANSparkMax(PortMap.INTAKE_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
    }

    public void extendArm() {
        this.armSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setSuckSpeed() {
        this.suckSpark.set(this.INTAKE_SPEED);
    }

    public void retractArm() {
        this.armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void initDefaultCommand(){

    }

}
