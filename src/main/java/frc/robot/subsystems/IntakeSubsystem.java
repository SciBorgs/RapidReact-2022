package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;

public class IntakeSubsystem implements Subsystem {

    private DoubleSolenoid armSolenoid;
    private CANSparkMax suckSpark;
    public DigitalInput limitSwitch;

    private final double INTAKE_SPEED = 0.5;

    public IntakeSubsystem() {
        this.armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortMap.INTAKE_ARM_FORWARD_CHANNEL, PortMap.INTAKE_ARM_REVERSE_CHANNEL);
        this.suckSpark = new CANSparkMax(PortMap.INTAKE_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        this.limitSwitch = new DigitalInput(PortMap.LIMIT_SWITCH_INTAKE);
    }

    public void extendArm() {
        this.armSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setSuckSpeed() {
        this.suckSpark.set(this.INTAKE_SPEED);
    }

    public void stopSuck() {
        this.suckSpark.set(0);
    }

    /* public void retractArm() {
        this.armSolenoid.set(DoubleSolenoid.Value.kReverse);
    } */

    public void initDefaultCommand(){

    }

    public double getIntakeSpeed() {
        return this.suckSpark.get();
    }

    public boolean getSwitchStatus() {
        return this.limitSwitch.get();
    }


}
