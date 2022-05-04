package frc.robot.subsystems;
import frc.robot.PortMap;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ClimberSubsystem implements Subsystem {
    private CANSparkMax telescope, arms;
   
    private static final double TELESCOPE_SPEED = 0.95;
    private static final double ARM_SPEED = 0.5;

    public ClimberSubsystem() {
        this.telescope = new CANSparkMax(PortMap.CLIMBER_TELESCOPE, MotorType.kBrushless);
        this.arms = new CANSparkMax(PortMap.CLIMBER_ARMS, MotorType.kBrushless);

        this.telescope.setIdleMode(IdleMode.kBrake);
        this.arms.setIdleMode(IdleMode.kBrake);
    }

    public void runTelescope(boolean reversed) {
        this.telescope.set(reversed ? -TELESCOPE_SPEED : TELESCOPE_SPEED);
    }

    public void stopTelescope() {
        this.telescope.set(0);
    }

    public void runArms(boolean reversed) {
        this.arms.set(reversed ? -ARM_SPEED : ARM_SPEED);
    }

    public void stopArms() {
        this.arms.set(0);
    }
}
