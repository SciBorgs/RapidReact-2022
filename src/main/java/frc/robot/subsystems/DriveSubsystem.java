package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.util.Util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
    public CANSparkMax lFront, lMiddle, lBack, rFront, rMiddle, rBack;
    private boolean invertedControl;
    private double speedLimit;

    public DriveSubsystem() {
        this.lFront  = new CANSparkMax(PortMap.LEFT_FRONT_SPARK,  MotorType.kBrushless);
        this.lMiddle = new CANSparkMax(PortMap.LEFT_MIDDLE_SPARK, MotorType.kBrushless);
        this.lBack   = new CANSparkMax(PortMap.LEFT_BACK_SPARK,   MotorType.kBrushless);

        this.rFront  = new CANSparkMax(PortMap.RIGHT_FRONT_SPARK,  MotorType.kBrushless);
        this.rMiddle = new CANSparkMax(PortMap.RIGHT_MIDDLE_SPARK, MotorType.kBrushless);
        this.rBack   = new CANSparkMax(PortMap.RIGHT_BACK_SPARK,   MotorType.kBrushless);

        lMiddle.follow(lFront);
        lBack.follow(lFront);

        rMiddle.follow(rFront);
        rBack.follow(rFront);

        lFront.setIdleMode(IdleMode.kCoast);
        lMiddle.setIdleMode(IdleMode.kCoast);
        lBack.setIdleMode(IdleMode.kCoast);

        rFront.setIdleMode(IdleMode.kCoast);
        rMiddle.setIdleMode(IdleMode.kCoast);
        rBack.setIdleMode(IdleMode.kCoast);

        this.speedLimit = 0.95;
        this.invertedControl = false;
    }

    public double getSpeedLimit() { return this.speedLimit; }
    public boolean getInvertedControl() { return this.invertedControl; }

    public void setSpeedLimit(double speedLimit) { this.speedLimit = speedLimit; }
    public void setInvertedControl(boolean inverted) { this.invertedControl = inverted; }

    private void setSpeedRaw(double left, double right) {
        lFront.set(left);
        rFront.set(right);
    }

    public void setSpeed(double left, double right) {
        if (invertedControl) {
            left *= -1.0;
            right *= -1.0;
        }
        setSpeedRaw(Util.normalize(left, this.speedLimit), -Util.normalize(right, this.speedLimit));
    }

    public void setSpeedForwardAngle(double forward, double angle) {
        // System.out.println(forward * (1 + angle));
        setSpeed(forward * (1 + angle), forward * (1 - angle)); // thank you zev
    }

    public void spinRobot(double speed) {
        setSpeed(-speed, speed);
    }

    public void driveRobot(Joystick leftJoystick, Joystick rightJoystick, double speedLimit) {
        double leftValue = -leftJoystick.getY();
        double rightValue = rightJoystick.getY();

        double thresholdToMove = 0.05;

        lFront.set(Math.abs(leftValue) > thresholdToMove ? leftValue : 0);
        rFront.set(Math.abs(rightValue) > thresholdToMove ? rightValue : 0);
    }
}