package frc.robot.subsystems;

import org.photonvision.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ta: area
// x: pitch

public class PhotonVisionSubsystem extends SubsystemBase {
    // returns photonvision network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("photonvision");
    }

    public double getTableData(NetworkTable table, String var) {
        return table.getEntry(var).getDouble(0.0);
    }
}
