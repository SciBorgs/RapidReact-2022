package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;

//things that aid the shuffleboard crew (nathanael, dani)
public class ShuffleUtil {
    public static void updateEntry(NetworkTableEntry entry, double newValue) {
        entry.setDouble(newValue);
    }
}
