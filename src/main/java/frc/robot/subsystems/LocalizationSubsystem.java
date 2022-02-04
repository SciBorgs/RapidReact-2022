package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciPigeon;

public class LocalizationSubsystem extends SubsystemBase {
    public SciPigeon pigeon;

    public Encoder leftEncoder, rightEncoder;

    public LocalizationSubsystem() {
        pigeon = new SciPigeon(PortMap.PIGEON_ID);
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(0, 1);
    }

    public double getX() {
        return -1;
    }

    public double getY() {
        return -1;
    }
    
    public double getAngle() {
        return pigeon.getAngle();
    }
}