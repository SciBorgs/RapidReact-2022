package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.robot.PortMap;

public class PneumaticsSubsystem implements Subsystem {
    private Compressor compressor;

    public PneumaticsSubsystem() {
        this.compressor = new Compressor(PortMap.COMPRESSOR, PneumaticsModuleType.CTREPCM);
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
