package frc.robot.subsystems;

import org.photonvision.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;


// ta: area
// x: pitch

public class PhotonVisionSubsystem extends SubsystemBase {
    public PhotonCamera photonCamera;

    public PhotonVisionSubsystem(){
        PhotonCamera photonCamera = new PhotonCamera("photonvision");
    }
    // returns photonvision network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("photonvision");
    }

    public double getTableData(NetworkTable table, String var) {
        return table.getEntry(var).getDouble(0.0);
    }
}
