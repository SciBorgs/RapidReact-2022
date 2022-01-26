package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
    public CANSparkMax lFront, lMiddle, lBack, rFront, rMiddle, rBack;

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
    }

    public void setSpeed(double left, double right) {
        lFront.set(left * 0.5);
        rFront.set(-right * 0.5);

        // System.out.println(left + "\t" + right);
    }

    public void setSpeedForwardAngle(double forward, double angle) {
        //System.out.println(forward * (1 + angle));
        setSpeed(forward * (1 + angle), forward * (1 - angle)); // thank you zev
    }
}