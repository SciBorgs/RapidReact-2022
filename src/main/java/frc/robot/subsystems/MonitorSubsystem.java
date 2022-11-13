package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MonitorSubsystem implements Subsystem {
  private PowerDistribution pdp;
  public ShuffleboardTab mainTab;
  public ShuffleboardTab currentTab;

  public MonitorSubsystem() {
    this.pdp = new PowerDistribution();
    mainTab = Shuffleboard.getTab("Main");
    mainTab.addNumber("Bus Voltage", pdp::getVoltage);
    mainTab.addNumber("Temperature", pdp::getTemperature);
    mainTab.addNumber("Current", pdp::getTotalCurrent);
    currentTab = Shuffleboard.getTab("Current");

    for (int i = 0; i < pdp.getNumChannels(); i++) {
      final Integer mi = Integer.valueOf(i); // Java moment
      currentTab.addNumber(
          "Channel " + (i + 1) + " Current",
          () -> {
            return pdp.getCurrent(mi);
          });
    }
  }
}
