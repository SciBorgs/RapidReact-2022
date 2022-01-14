package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;

public class Hopper extends Subsystem {

    CANSparkMax in;
    CANSparkMax up;

    public Hopper(CANSparkMax in, CANSparkMax up) {
        this.in = in;
        this.up = up;
    }

    public void setInSpeed(double speed) {
        in.set(speed);
    }

    public void setOutSpeed(double speed) {
        out.set(speed);
    }

    public double getInSpeed() {
        return in.get();
    }

    public double getOutSpeed() {
        return out.get();
    }
}