package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax horizontal, vertical;

    public TurretSubsystem() {

    }
    
    public void aim(int x, int y) {

    }

    public Object find() {
        return new Object();
    }
}
