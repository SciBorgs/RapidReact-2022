package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.util.Util;

public class HopperSubsystem implements Subsystem {

    private CANSparkMax suck;
    private CANSparkMax elevator;

    private final double HOPPER_SPEED = 0.35;
    private final double ELEVATOR_SPEED = 0.5;
    
    private final double MAX_SPEED = 0.8;

    // public ShuffleboardTab hopperTab;

    public HopperSubsystem() {
        this.suck = new CANSparkMax(PortMap.HOPPER_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        this.elevator = new CANSparkMax(PortMap.HOPPER_ELEVATOR_SPARK, CANSparkMax.MotorType.kBrushless);

        // hopperTab = Shuffleboard.getTab("Hopper");
        // hopperTab.addNumber("Suck Speed", this::getSuckSpeed);
        // hopperTab.addNumber("Elevator Speed", this::getElevatorSpeed);
    }

    public void startSuck() {
        this.suck.set(HOPPER_SPEED);
    }

    public void startSuck(double newSpeed) {
        this.suck.set(MathUtil.clamp(newSpeed, -MAX_SPEED, MAX_SPEED));
    }

    public void stopSuck() {
        this.suck.set(0);
    }

    public void startElevator() {
        this.elevator.set(ELEVATOR_SPEED);
    }
    
    public void startElevator(double newSpeed) {
        this.elevator.set(MathUtil.clamp(newSpeed, -MAX_SPEED, MAX_SPEED));
    }

    public void stopElevator() {
        this.elevator.set(0);
    }

    public double getSuckSpeed() {
        return suck.get();
    }

    public double getElevatorSpeed() {
        return elevator.get();
    }
}
