package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.PortMap;

public class IntakeSubsystem extends SubsystemBase {

  private DoubleSolenoid armSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          PortMap.Intake.ARM_CHANNELS[0],
          PortMap.Intake.ARM_CHANNELS[1]);
  private CANSparkMax suckSpark =
      new CANSparkMax(PortMap.Intake.SUCK_SPARK, CANSparkMax.MotorType.kBrushless);

  // detecting when ball is in intake
  private DigitalInput limitSwitch = new DigitalInput(PortMap.Intake.LIMIT_SWITCH);
  private Debouncer ballFilter =
      new Debouncer(IntakeConstants.DEBOUNCE_TIME, DebounceType.kFalling);
  private boolean hasBall = false;

  private ShuffleboardTab tab;
  private SimpleWidget intakeSpeedWidget;

  public IntakeSubsystem() {
    this.armSolenoid.set(DoubleSolenoid.Value.kForward);

    this.tab = Shuffleboard.getTab("Intake");
    this.tab.addNumber("Intake Suck Speed", this::getIntakeSpeed);
    this.tab.addNumber("Intake Suck Applied Output", this.suckSpark::getAppliedOutput);
    this.tab.addNumber("Intake Suck RPM", this.suckSpark.getEncoder()::getVelocity);

    // this.intakeSpeedWidget = this.tab.add("Intake Suck Set", intakeSpeed);

    // this.intakeSpeedWidget.getEntry().addListener(event -> {
    //     this.startSuck(event.getEntry().getDouble(this.intakeSpeed));
    // }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  public void toggleArm() {
    armSolenoid.toggle();
  }

  public void extendArm() {
    armSolenoid.set(Value.kReverse);
  }

  public void retractArm() {
    armSolenoid.set(Value.kForward);
  }

  public void startSuck() {
    startSuck(IntakeConstants.INTAKE_SPEED);
  }

  public void startSuck(double speed) {
    suckSpark.set(MathUtil.clamp(speed, -1, 1));
  }

  public void reverseSuck() {
    reverseSuck(-IntakeConstants.INTAKE_SPEED);
  }

  public void reverseSuck(double speed) {
    suckSpark.set(MathUtil.clamp(speed, -1, 1));
  }

  public void stopSuck() {
    suckSpark.stopMotor();
  }

  public double getIntakeSpeed() {
    return this.suckSpark.get();
  }

  public boolean hasBall() {
    return hasBall;
  }

  @Override
  public void periodic() {
    hasBall = ballFilter.calculate(limitSwitch.get());
  }
}
