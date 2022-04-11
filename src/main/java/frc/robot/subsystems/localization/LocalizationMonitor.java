package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LocalizationMonitor {
    private LocalizationSubsystem localizationSubsystem;
    private ShuffleboardTab tab;

    public LocalizationMonitor(LocalizationSubsystem localizationSubsystem) {
        this.localizationSubsystem = localizationSubsystem;
        tab = Shuffleboard.getTab("localization");
        tab.addDoubleArray("pose",          localizationSubsystem::get);
        tab.addDoubleArray("particles",     localizationSubsystem.particleFilter::getFlat);
        tab.addDoubleArray("meanParticle",  localizationSubsystem.particleFilter::getMeanParticle);
    }
}
