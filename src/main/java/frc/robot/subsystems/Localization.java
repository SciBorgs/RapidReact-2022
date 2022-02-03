package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciPigeon;

public class Localization extends SubsystemBase {
    public SciPigeon pigeon;

    public Encoder leftEncoder, rightEncoder;

    public Localization() {
        pigeon = new SciPigeon(PortMap.PIGEON_ID);
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(0, 1);
    }

    public int getX() {
        return -1;
    }

    public int getY() {
        return -1;
        
    }
    
}