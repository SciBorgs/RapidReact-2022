package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RumbleSubsystem extends SubsystemBase {

    private XboxController controller;

    public RumbleSubsystem(XboxController controller) {
        this.controller = controller;
    }

    public void rumble() {
        controller.setRumble(RumbleType.kLeftRumble, 0.5);
        controller.setRumble(RumbleType.kRightRumble, 0.5);
    }

    public void stopRumble() {
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);

    }

}