package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.PortMap;
import frc.robot.Robot;

public class PneumaticsSubsystem implements Subsystem {
    private Compressor compressor;    
    public  ShuffleboardTab mainTab;

    public PneumaticsSubsystem() {
        this.compressor = new Compressor(PortMap.COMPRESSOR, PneumaticsModuleType.CTREPCM);
        mainTab = Shuffleboard.getTab("Main");
        mainTab.addBoolean("Compressor Running", this::getStatus);
    }

    public void start() {
        this.compressor.enableDigital();
    }

    public void stop() {
        this.compressor.disable();
    }

    public boolean getStatus() {
        return this.compressor.enabled();
    }
}