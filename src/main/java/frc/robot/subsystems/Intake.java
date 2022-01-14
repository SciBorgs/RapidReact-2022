package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;



public class Intake extends Subsystem {

    CANSparkMax armSpark;
    CANSparkMax suckSpark;

    public Intake() {
        this.armSpark = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
        this.suckSpark = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
    }

    public void setArmSpeed(double speed) {
        armSpark.set(speed);
    }

    public void setSuckSpeed(double speed) {
        suckSpark.set(speed);
    }

    @Override
    public void initDefaultCommand(){}


}