package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {

    // returns limelight network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    
    // returns the val`ue of an entry in the limelight network table
    public double getTableData(NetworkTable table, String var) {
        return table.getEntry(var).getDouble(0.0);
    }

    // sets a camera paramter
    public void setCameraParams(String param, int setting) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(param).setValue(setting);
    }

    public void setCameraParams(String param, double setting) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(param).setValue(setting);
    }
    
}
