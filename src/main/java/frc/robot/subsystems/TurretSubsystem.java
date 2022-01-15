package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax turretMotor;
    //Depends on how many motors there are on turret, but Im thinking there is only one for now.

    public TurretSubsystem() {

    }

    public void aim(int x, int y) {

    }

    public void setTurretSpeed(double speed) {
        this.turretMotor.set(speed);
    }

    public Object find() {
        return new Object();
    }
}
