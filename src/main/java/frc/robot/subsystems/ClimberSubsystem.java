package frc.robot.subsystems;
import frc.robot.PortMap;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class ClimberSubsystem implements Subsystem {
    private CANSparkMax climberArm;
    private CANSparkMax hookMotor;
    
    private ShuffleboardTab tab;
    private NetworkTableEntry rotations, armEntry, hookMotorEntry;
    
    private final double CLIMBER_ARM_SPEED = 0.5;
    private final double HOOK_MOTOR_SPEED = 0.2;

    public ClimberSubsystem() {
        this.climberArm = new CANSparkMax(PortMap.CLIMBER_ARM, MotorType.kBrushless);
        this.hookMotor = new CANSparkMax(PortMap.HOOK_MOTOR, MotorType.kBrushless);
        
        tab = Shuffleboard.getTab("Climber");
        rotations = tab.add("Number of Rotations", 0.0).getEntry();
        armEntry = tab.add("Climber Arm Speed", 0.0).getEntry();
        hookMotorEntry = tab.add("Hook Motor Speed", 0.0).getEntry();
    }

    public void extendClimberArm() {
        this.climberArm.set(this.CLIMBER_ARM_SPEED);
    }

    public void stopClimberArm() {
        this.climberArm.set(0);
    }

    public void setHookMotorSpeed() {
        this.hookMotor.set(this.HOOK_MOTOR_SPEED);
    }

    public void stopHookMotor() {
        this.hookMotor.set(0);
    }

    public void retractClimberArm() {
        this.climberArm.set(-(this.CLIMBER_ARM_SPEED));
    }

    public void initDefaultCommand(){

    }

}
