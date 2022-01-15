package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.button.*;

public class OI {
        public Button intakeBalls, lowerIntakeArms, StartHopper;

        public OI() {
                this.intakeBalls = new Button();
                this.intakeBalls.whenHeld(new IntakeBalls());

                this.lowerIntakeArms = new Button();
                this.lowerIntakeArms.whenPressed(new LowerIntakeArm().withTimeout(0.2));

                this.StartHopper = new Button();
                this.StartHopper.whenPressed(new StartHopper());

        }
}
