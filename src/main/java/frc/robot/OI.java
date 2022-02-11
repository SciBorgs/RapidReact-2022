package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.button.*;

public class OI {
        public Button lowerIntakeArms, StartHopper, togglePneumatics;

        public OI() {
                this.lowerIntakeArms = new Button();
                this.lowerIntakeArms.whenPressed(new LowerIntakeArm());

                this.StartHopper = new Button();
                this.StartHopper.whenPressed(new StartHopper());

                this.togglePneumatics = new Button();
                this.togglePneumatics.toggleWhenPressed(new TogglePneumatics());

        }
}

