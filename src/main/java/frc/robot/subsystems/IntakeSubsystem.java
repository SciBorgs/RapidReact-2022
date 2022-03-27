package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;
import frc.robot.Robot;

public class IntakeSubsystem implements Subsystem {

    private DoubleSolenoid armSolenoid; // solenoid used for extending and retracting intake arm
    private CANSparkMax suckSpark; // motor used for intaking balls
    public  DigitalInput limitSwitch; // limit switch used for detecting when ball in intake

    public  ShuffleboardTab intakeTab;

    private final double INTAKE_SPEED = 0.5;

    public IntakeSubsystem() {
        this.armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortMap.INTAKE_ARM_FORWARD_CHANNEL, PortMap.INTAKE_ARM_REVERSE_CHANNEL); 
        this.suckSpark = new CANSparkMax(PortMap.INTAKE_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        // this.suckSpark.setInverted(true); // invert the motor
        this.limitSwitch = new DigitalInput(PortMap.LIMIT_SWITCH_INTAKE);

        intakeTab = Shuffleboard.getTab("Intake");
        intakeTab.addBoolean("Intake Running", this::getIntakeRunning);
        intakeTab.addBoolean("Limit Switch Set", this::getSwitchStatus);
        intakeTab.addString("Solenoid Arm Status", this::getSolenoidArmStatus);
    }

    public void extendArm() { 
        this.armSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void retractArm() {
        this.armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void startSuck() {
        this.suckSpark.set(this.INTAKE_SPEED);
    }

    public void stopSuck() {
        this.suckSpark.set(0);
    }

    public double getIntakeSpeed() {
        return this.suckSpark.get();
    }

    public boolean getIntakeRunning() {
        return (Math.abs(getIntakeSpeed()) < 0.04);
    }

    public boolean getSwitchStatus() {
        return this.limitSwitch.get();
    }

    public String getSolenoidArmStatus() { 
        return this.armSolenoid.get().name().substring(1);
    }
    

}
