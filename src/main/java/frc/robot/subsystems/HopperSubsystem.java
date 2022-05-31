package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.PortMap;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.Blockable;

@Blockable
public class HopperSubsystem implements Subsystem {

    private CANSparkMax suck;
    private CANSparkMax elevator;

    private double suckSpeed;
    private double elevatorSpeed;

    public HopperSubsystem() {
        this.suck = new CANSparkMax(PortMap.Hopper.SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        this.elevator = new CANSparkMax(PortMap.Hopper.ELEVATOR_SPARK, CANSparkMax.MotorType.kBrushless);

        suckSpeed = 0;
        elevatorSpeed = 0;
    }

    public void startSuck() {
        suckSpeed = HopperConstants.SUCK_SPEED;
    }

    public void startSuck(double newSpeed) {
        suckSpeed = MathUtil.clamp(newSpeed, -HopperConstants.MAX_SPEED, HopperConstants.MAX_SPEED);
    }

    public void stopSuck() {
        suckSpeed = 0;
    }

    public void startElevator() {
        suckSpeed = HopperConstants.SUCK_SPEED;
    }
    
    public void startElevator(double newSpeed) {
        elevatorSpeed = MathUtil.clamp(newSpeed, -HopperConstants.MAX_SPEED, HopperConstants.MAX_SPEED);
    }

    public void stopElevator() {
        elevatorSpeed = 0;
    }

    public double getSuckSpeed() {
        return suck.get();
    }

    public double getElevatorSpeed() {
        return elevator.get();
    }

    public double getTargetSuckSpeed() {
        return suckSpeed;
    }

    public double getTargetElevatorSpeed() {
        return elevatorSpeed;
    }
    
    @Override
    public void periodic() {
        elevator.set(elevatorSpeed);
        suck.set(suckSpeed);
    }
}
