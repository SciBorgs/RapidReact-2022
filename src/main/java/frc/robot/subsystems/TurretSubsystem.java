package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.util.PID;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax turretMotor = new CANSparkMax(PortMap.TURRET_SPARK, MotorType.kBrushless);
    //Depends on how many motors there are on turret, but Im thinking there is only one for now.
    private PID pid = new PID(1, 1, 1);

    public void turn() {
        double tx = Robot.limeLightSubsystem.getTableData(Robot.limeLightSubsystem.getTable(), "tx");
        double speed = pid.getOutput(0, tx);
        setTurretSpeed(speed);
    }

    public void setTurretSpeed(double speed) {
        this.turretMotor.set(speed);
    }
}
