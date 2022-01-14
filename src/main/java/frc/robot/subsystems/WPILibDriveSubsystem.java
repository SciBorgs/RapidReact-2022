package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SpeedController;

public class WPILibDriveSubsystem extends SubsystemBase {
    protected DifferentialDrive drive;
    private SpeedController lDrive, rDrive;

    public WPILibDriveSubsystem() {
        this.lDrive = new SpeedControllerGroup(
            new CANSparkMax(PortMap.LEFT_FRONT_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.LEFT_MIDDLE_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.LEFT_BACK_SPARK, MotorType.kBrushless));
        this.rDrive = new SpeedControllerGroup(
            new CANSparkMax(PortMap.RIGHT_FRONT_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.RIGHT_MIDDLE_SPARK, MotorType.kBrushless),
            new CANSparkMax(PortMap.RIGHT_BACK_SPARK, MotorType.kBrushless));
        this.drive = new DifferentialDrive(lDrive, rDrive);
    }

    public void teleopPeriodic() {
        drive.tankDrive(m_leftStick.getY(), m_rightStick.getY());
    }
}