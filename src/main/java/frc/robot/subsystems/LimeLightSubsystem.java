package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {

    // returns limelight network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    
    // returns the value of an entry in the limelight network table
    public double getTableData(NetworkTable table, String var) {
        return table.getEntry(var).getDouble(0.0);
    }

    // sets a camera paramter
    public void setCameraParams(NetworkTable table, String param, int setting) {
        table.getEntry(param).setNumber(setting);
    }

    public void setCameraParams(NetworkTable table, String param, double setting) {
        table.getEntry(param).setValue(setting);
    }
    
}
