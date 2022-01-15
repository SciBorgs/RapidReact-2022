package frc.robot.subsystems;
import frc.robot.PortMap;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem implements Subsystem {
    private CANSparkMax climberArm;
    private final double CLIMBER_ARM_SPEED = 0.5;

    public ClimberSubsystem() {
        this.climberArm = new CANSparkMax(PortMap.CLIMBER_ARM, MotorType.kBrushless);
    }

    public void setClimberArmSpeed() {
        this.climberArm.set(this.CLIMBER_ARM_SPEED);
    }

    public void stopClimberArm() {
        this.climberArm.set(0);
    }

    public void initDefaultCommand(){

    }

}
