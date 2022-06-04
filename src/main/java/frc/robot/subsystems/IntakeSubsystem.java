package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.PortMap;
import frc.robot.util.BallCounter;
import frc.robot.util.Blockable;

@Blockable
public class IntakeSubsystem extends SubsystemBase implements BallCounter {

    private DoubleSolenoid armSolenoid; // solenoid used for extending and retracting intake arm
    private CANSparkMax suckSpark; // motor used for intaking balls

    private DigitalInput limitSwitch; // limit switch used for detecting when ball in intake
    private boolean lastLimit;
    private long lastFallingEdge;

    private double intakeSpeed;

    private ShuffleboardTab mainTab;
    private SimpleWidget intakeSpeedWidget;

    public IntakeSubsystem() {
        this.armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortMap.Intake.ARM_CHANNELS[0], PortMap.Intake.ARM_CHANNELS[1]); 
        this.suckSpark = new CANSparkMax(PortMap.Intake.SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        // this.suckSpark.setInverted(true); // invert the motor
        this.limitSwitch = new DigitalInput(PortMap.Intake.LIMIT_SWITCH);

        this.armSolenoid.set(DoubleSolenoid.Value.kReverse);

        this.intakeSpeed = 0;

        
        // this.mainTab = Shuffleboard.getTab("Intake");
        // this.mainTab.addNumber("Intake Suck Speed", this::getIntakeSpeed);
        // this.mainTab.addNumber("Intake Suck Applied Output", this.suckSpark::getAppliedOutput);
        // this.mainTab.addNumber("Intake Suck RPM", this.suckSpark.getEncoder()::getVelocity);


        // this.intakeSpeedWidget = this.mainTab.add("Intake Suck Set", intakeSpeed);

        // this.intakeSpeedWidget.getEntry().addListener(event -> {
        //     this.startSuck(event.getEntry().getDouble(this.intakeSpeed));
        // }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void updateBallCounter(){

        if(lastLimit && !this.getLimitSwitchState()) //if on falling edge, note and end (falling edge = turning off)
            lastFallingEdge = System.currentTimeMillis();

        if(!lastLimit && this.getLimitSwitchState()) //if on rising edge, measure time from last rising edge (rising edge = turning on)
            if(System.currentTimeMillis() - lastFallingEdge > IntakeConstants.WAIT_TIME) //so we know ball did not shake around in intake
            increment();

        lastLimit = this.getLimitSwitchState();
    }

    public boolean getLimitSwitchState(){
        return limitSwitch.get();
    }

    public void toggleArm() { 
        this.armSolenoid.toggle();
    }

    public void startSuck() {
        this.intakeSpeed = IntakeConstants.INTAKE_SPEED;
    }

    public void startSuck(double speed) {
        this.intakeSpeed = MathUtil.clamp(speed, -1, 1);
    }

    public void stopSuck() {
        this.intakeSpeed = 0;
    }

    public double getIntakeSpeed() {
        return this.suckSpark.get();
    }

    @Override
    public void periodic() {
        suckSpark.set(intakeSpeed);
        updateBallCounter();
    }
}
