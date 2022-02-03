package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciPigeon;

public class Localization extends SubsystemBase {
    public SciPigeon pigeon;

    public Localization() {
        pigeon = new SciPigeon(PortMap.PIGEON_ID);
    }

    
}