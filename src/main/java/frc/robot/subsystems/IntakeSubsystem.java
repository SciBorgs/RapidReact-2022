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
import frc.robot.util.Blockable;

@Blockable
public class IntakeSubsystem implements Subsystem {

    private DoubleSolenoid armSolenoid; // solenoid used for extending and retracting intake arm
    private CANSparkMax suckSpark; // motor used for intaking balls
    private boolean lastLimit = false;

    public DigitalInput limitSwitch; // limit switch used for detecting when ball in intake
    public int amountOfBalls = 0;
    public final int WAIT_TIME = 1000; //in miliseconds
    public long lastActivated = 0;
    
    private final double INTAKE_SPEED = 0.5;

    public IntakeSubsystem() {
        this.armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortMap.INTAKE_ARM_FORWARD_CHANNEL, PortMap.INTAKE_ARM_REVERSE_CHANNEL); 
        this.suckSpark = new CANSparkMax(PortMap.INTAKE_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        // this.suckSpark.setInverted(true); // invert the motor
        this.limitSwitch = new DigitalInput(PortMap.LIMIT_SWITCH_INTAKE);
        
    }
    // ONLY WORKS IF LIMIT SWITCH IS IN INTAKE AND NOT HOPPER balsucker//
    public void updateBallCounter(){

        // // Should also work
        // if(lastLimit && !this.getLimitSwitchState()) //if on falling edge, note and end (falling edge = turning off)
        //     lastFallingEdge = System.currentTimeMillis();

        // if(!lastLimit && this.getLimitSwitchState()) //if on rising edge, measure time from last rising edge (rising edge = turning on)
        //     if(System.currentTimeMillis() - lastFallingEdge > WAIT_TIME) //so we know ball did not shake around in intake
        //         amountOfBalls += 1;

        // lastLimit = this.getLimitSwitchState();
        

        if (this.getLimitSwitchState()) {
            if (System.currentTimeMillis() - lastActivated > WAIT_TIME) {
                amountOfBalls++;
            }
            lastActivated = System.currentTimeMillis();
        }
    }

    public boolean getLimitSwitchState(){
        return limitSwitch.get();
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
    
}
