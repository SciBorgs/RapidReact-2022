package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.util.Blockable;
import frc.robot.util.Counter;

@Blockable
public class IntakeSubsystem extends SubsystemBase implements Counter {

    private DoubleSolenoid armSolenoid; // solenoid used for extending and retracting intake arm
    private CANSparkMax suckSpark; // motor used for intaking balls

    private DigitalInput limitSwitch; // limit switch used for detecting when ball in intake
    private boolean lastLimit;
    private long lastFallingEdge;
    private final int WAIT_TIME = 1000; //in miliseconds
    
    private final double INTAKE_SPEED = 0.5;

    public IntakeSubsystem() {
        this.armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortMap.INTAKE_ARM_FORWARD_CHANNEL, PortMap.INTAKE_ARM_REVERSE_CHANNEL); 
        this.suckSpark = new CANSparkMax(PortMap.INTAKE_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        // this.suckSpark.setInverted(true); // invert the motor
        this.limitSwitch = new DigitalInput(PortMap.LIMIT_SWITCH_INTAKE);

        this.armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // TODO remove, move updating to periodic
    public void updateBallCounter(){

        if(lastLimit && !this.getLimitSwitchState()) //if on falling edge, note and end (falling edge = turning off)
            lastFallingEdge = System.currentTimeMillis();

        if(!lastLimit && this.getLimitSwitchState()) //if on rising edge, measure time from last rising edge (rising edge = turning on)
            if(System.currentTimeMillis() - lastFallingEdge > WAIT_TIME) //so we know ball did not shake around in intake
            // amountOfBalls += 1;

        lastLimit = this.getLimitSwitchState();
    }

    public boolean getLimitSwitchState(){
        return limitSwitch.get();
    }

    public void toggleArm() { 
        this.armSolenoid.toggle();
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

    // counter
    @Override
    public void increment() {
        count.increment();
        
    }

    @Override
    public void decrement() {
        count.decrement();
        
    }

    @Override
    public int get() {
        return count.get();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    
}
