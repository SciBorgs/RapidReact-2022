package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class WPILibDriveSubsystem extends SubsystemBase {
    protected DifferentialDrive drive;
    private MotorController lDrive, rDrive;

    public WPILibDriveSubsystem() {
        this.lDrive = new MotorControllerGroup(
            new CANSparkMax(PortMap.LEFT_FRONT_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.LEFT_MIDDLE_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.LEFT_BACK_SPARK, MotorType.kBrushless));
        this.rDrive = new MotorControllerGroup(
            new CANSparkMax(PortMap.RIGHT_FRONT_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.RIGHT_MIDDLE_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.RIGHT_BACK_SPARK, MotorType.kBrushless));
        this.drive = new DifferentialDrive(lDrive, rDrive);
    }

    public void teleopPeriodic() {
        drive.tankDrive(Robot.oi.joystickLeft.getY(), Robot.oi.joystickRight.getY());
    }
}