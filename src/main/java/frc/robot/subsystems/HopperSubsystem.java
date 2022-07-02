package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

    private CANSparkMax suck;
    private CANSparkMax elevator;

    private double suckSpeed;
    private double elevatorSpeed;
    private ShuffleboardTab mainTab;
    private SimpleWidget hopperSuckSpeed;
    private SimpleWidget hopperElevatorSpeed;

    public HopperSubsystem() {
        this.suck = new CANSparkMax(PortMap.Hopper.SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
        this.elevator = new CANSparkMax(PortMap.Hopper.ELEVATOR_SPARK, CANSparkMax.MotorType.kBrushless);
        // this.elevator.setInverted(true);

        suckSpeed = 0;
        elevatorSpeed = 0;

        this.mainTab = Shuffleboard.getTab("Hopper");
        this.mainTab.addNumber("Hopper Suck Speed", this::getTargetSuckSpeed);
        this.mainTab.addNumber("Hopper Suck Applied Output", this.suck::getAppliedOutput);
        this.mainTab.addNumber("Hopper Suck RPM", this.suck.getEncoder()::getVelocity);

        
        this.mainTab.addNumber("Hopper Elevator Speed", this::getTargetElevatorSpeed);
        this.mainTab.addNumber("Hopper Elevator Applied Output", this.elevator::getAppliedOutput);
        this.mainTab.addNumber("Hopper Elevator RPM", this.elevator.getEncoder()::getVelocity);

        this.hopperSuckSpeed = this.mainTab.add("Hopper Suck Set", suckSpeed);
        this.hopperElevatorSpeed = this.mainTab.add("Hopper Elevator Set", elevatorSpeed);

        this.hopperSuckSpeed.getEntry().addListener(event -> {
            this.startSuck(event.getEntry().getDouble(suckSpeed));
        }, EntryListenerFlags.kUpdate);

        this.hopperElevatorSpeed.getEntry().addListener(event -> {
            this.startElevator(event.getEntry().getDouble(elevatorSpeed));
        }, EntryListenerFlags.kUpdate);
    }

    public void startSuck() {
        suckSpeed = HopperConstants.SUCK_SPEED;
    }

    public void startSuck(double newSpeed) {
        suckSpeed = MathUtil.clamp(newSpeed, -HopperConstants.MAX_SPEED, HopperConstants.MAX_SPEED);
    }

    public void reverseSuck() {
        suckSpeed = -HopperConstants.SUCK_SPEED;
    }

    public void reverseSuck(double newSpeed) {
        suckSpeed = -MathUtil.clamp(newSpeed, -HopperConstants.MAX_SPEED, HopperConstants.MAX_SPEED);
    }

    public void stopSuck() {
        suckSpeed = 0;
    }

    public void startElevator() {
        elevatorSpeed = HopperConstants.ELEVATOR_SPEED;
    }
    
    public void startElevator(double newSpeed) {
        System.out.println("Setting Elevator");
        elevatorSpeed = MathUtil.clamp(newSpeed, -HopperConstants.MAX_SPEED, HopperConstants.MAX_SPEED);
        System.out.println("Clamped to " + elevatorSpeed);
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
        // System.out.println("elvatorSpeed: " + elevatorSpeed + " suckSpeed: " + suckSpeed);
        elevator.set(elevatorSpeed);
        // System.out.println("elvatorSpeed: " + elevatorSpeed);
        suck.set(suckSpeed);
    }
}
