package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
    public CANSparkMax lFront, lMiddle, lBack, rFront, rMiddle, rBack;

    public DriveSubsystem() {
        this.lFront = new CANSparkMax(PortMap.LEFT_FRONT_SPARK, MotorType.kBrushless);
        this.lMiddle = new CANSparkMax(PortMap.LEFT_MIDDLE_SPARK, MotorType.kBrushless);
        this.lBack = new CANSparkMax(PortMap.LEFT_BACK_SPARK, MotorType.kBrushless);

        this.rFront = new CANSparkMax(PortMap.RIGHT_FRONT_SPARK, MotorType.kBrushless);
        this.rMiddle = new CANSparkMax(PortMap.RIGHT_MIDDLE_SPARK, MotorType.kBrushless);
        this.rBack = new CANSparkMax(PortMap.RIGHT_BACK_SPARK, MotorType.kBrushless);

        lMiddle.follow(lFront);
        lBack.follow(lFront);
        
        rMiddle.follow(rFront);
        rBack.follow(rFront);
    }

    public void moveRobot(double left, double right) {
        lFront.set(left);
        rFront.set(right);
    }

    public void setSpeedTankForwardAngle(double forward, double angularOffset){
        
    }
}