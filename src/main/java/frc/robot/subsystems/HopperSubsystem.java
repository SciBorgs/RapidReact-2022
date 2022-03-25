package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.CANSparkMax;

import frc.robot.PortMap;

public class HopperSubsystem implements Subsystem {

    private CANSparkMax suck;
    private CANSparkMax elevator;

    private final double HOPPER_SPEED = 0.01;
    private final double ELEVATOR_SPEED = 0.01;
    
    private final double MAX_SPEED = 0.1;

    public HopperSubsystem() {
        this.suck = new CANSparkMax(PortMap.HOPPER_SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        this.elevator = new CANSparkMax(PortMap.HOPPER_ELEVATOR_SPARK, CANSparkMax.MotorType.kBrushless);
    }

    public void startSuck() {
        startSuck(HOPPER_SPEED);
    }

    public void startSuck(double newSpeed) {
        this.suck.set(Math.min(Math.max(newSpeed, -MAX_SPEED), MAX_SPEED));
    }

    public void stopSuck() {
        this.suck.set(0);
    }

    public void startElevator() {
        this.startElevator(ELEVATOR_SPEED);
    }
    
    public void startElevator(double newSpeed) {
        this.elevator.set(Math.min(Math.max(newSpeed, -MAX_SPEED), MAX_SPEED));
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
