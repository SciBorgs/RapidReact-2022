package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.util.ShuffleUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
    public CANSparkMax lFront, lMiddle, lBack, rFront, rMiddle, rBack;
    private ShuffleboardTab DriveTab;
    private NetworkTableEntry lFrontEntry, lMiddleEntry, lBackEntry, rFrontEntry, rMiddleEntry, rBackEntry;

    public DriveSubsystem() {
        this.lFront = new CANSparkMax(PortMap.LEFT_FRONT_SPARK, MotorType.kBrushless);
        // this.lMiddle = new CANSparkMax(PortMap.LEFT_MIDDLE_SPARK, MotorType.kBrushless);
        this.lBack = new CANSparkMax(PortMap.LEFT_BACK_SPARK, MotorType.kBrushless);

        this.rFront = new CANSparkMax(PortMap.RIGHT_FRONT_SPARK, MotorType.kBrushless);
        // this.rMiddle = new CANSparkMax(PortMap.RIGHT_MIDDLE_SPARK, MotorType.kBrushless);
        this.rBack = new CANSparkMax(PortMap.RIGHT_BACK_SPARK, MotorType.kBrushless);

        // lMiddle.follow(lFront);
        lBack.follow(lFront);
        
        // rMiddle.follow(rFront);
        rBack.follow(rFront);

        lFront.setIdleMode(IdleMode.kCoast);
        // lMiddle.setIdleMode(IdleMode.kCoast);
        lBack.setIdleMode(IdleMode.kCoast);
        rFront.setIdleMode(IdleMode.kCoast);
        // rMiddle.setIdleMode(IdleMode.kCoast);
        rBack.setIdleMode(IdleMode.kCoast);

        //NOTE: It appears that only the left and right front motors are ever set -Shuffleboard Duo
        DriveTab = Shuffleboard.getTab("Drivetrain");
        lFrontEntry = DriveTab.add("Left Front Motor Speed", 0).getEntry();
        lMiddleEntry = DriveTab.add("Left Middle Motor Speed", 0).getEntry();
        lBackEntry = DriveTab.add("Left Back Motor Speed", 0).getEntry();
        rFrontEntry = DriveTab.add("Right Back Motor Speed", 0).getEntry();
        rMiddleEntry = DriveTab.add("Right Middle Motor Speed", 0).getEntry();
        rBackEntry =  DriveTab.add("Right Back Motor Speed", 0).getEntry();
    }

    public void setSpeed(double left, double right) {
        lFront.set(-left * 0.8);
        rFront.set(right * 0.8);

        // System.out.println(left + "\t" + right);
    }

    public void setSpeedForwardAngle(double forward, double angle) {
        //System.out.println(forward * (1 + angle));
        setSpeed(forward * (1 + angle), forward * (1 - angle)); // thank you zev
    }
}