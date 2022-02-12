package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;

public class HopperSubsystem implements Subsystem {

    private CANSparkMax suck;
    private CANSparkMax elevator;

    private final double HOPPER_SPEED = 0.5;
    private final double ELEVATOR_SPEED = HOPPER_SPEED;

    public HopperSubsystem() {
        this.suck = new CANSparkMax(PortMap.HOPPER_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        this.elevator = new CANSparkMax(PortMap.HOPPER_ELEVATOR_SPARK, CANSparkMax.MotorType.kBrushless);
    }

    public void setSuckSpeed() {
        this.suck.set(this.HOPPER_SPEED);
    }

    public void setElevatorSpeed() {
        this.elevator.set(this.ELEVATOR_SPEED);
    }

    public void stopSuck() {
        this.suck.set(0);
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
