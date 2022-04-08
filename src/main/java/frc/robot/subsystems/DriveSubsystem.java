package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciSpark;
import frc.robot.util.Util;

import com.revrobotics.CANSparkMax.IdleMode;

public class DriveSubsystem extends SubsystemBase {
    public SciSpark lFront, lMiddle, lBack, rFront, rMiddle, rBack;
    private boolean invertedControl;
    private double speedLimit;

    public DriveSubsystem() {
        this.lFront  = new SciSpark(PortMap.LEFT_FRONT_SPARK);
        this.lMiddle = new SciSpark(PortMap.LEFT_MIDDLE_SPARK);
        this.lBack   = new SciSpark(PortMap.LEFT_BACK_SPARK);

        this.rFront  = new SciSpark(PortMap.RIGHT_FRONT_SPARK);
        this.rMiddle = new SciSpark(PortMap.RIGHT_MIDDLE_SPARK);
        this.rBack   = new SciSpark(PortMap.RIGHT_BACK_SPARK);

        lMiddle.follow(lFront);
        lBack.follow(lFront);

        rMiddle.follow(rFront);
        rBack.follow(rFront);

        lFront.setIdleMode(IdleMode.kBrake);
        lMiddle.setIdleMode(IdleMode.kBrake);
        lBack.setIdleMode(IdleMode.kBrake);

        rFront.setIdleMode(IdleMode.kBrake);
        rMiddle.setIdleMode(IdleMode.kBrake);
        rBack.setIdleMode(IdleMode.kBrake);

        this.speedLimit = 0.95;
        this.invertedControl = false;
    }

    public double getSpeedLimit() { return this.speedLimit; }
    public boolean getInvertedControl() { return this.invertedControl; }

    public void setSpeedLimit(double speedLimit) { this.speedLimit = speedLimit; }
    public void setInvertedControl(boolean inverted) { this.invertedControl = inverted; }

    private void setSpeedRaw(double left, double right) {
        System.out.println("speeds: " + left + " " + right);
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
        double leftValue = leftJoystick.getY();
        double rightValue = -rightJoystick.getY();

        double thresholdToMove = 0.05;

        lFront.set(Math.abs(leftValue) > thresholdToMove ? leftValue : 0);
        rFront.set(Math.abs(rightValue) > thresholdToMove ? rightValue : 0);
    }

    public void driveRobot(XboxController xboxController, double speedLimit) {
        double leftValue  = xboxController.getLeftY();
        double rightValue = xboxController.getRightY();

        double thresholdToMove = 0.05;

        lFront.set(Math.abs(leftValue) > thresholdToMove ? leftValue : 0);
        rFront.set(Math.abs(rightValue) > thresholdToMove ? rightValue : 0);
    }
}