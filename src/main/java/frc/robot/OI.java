package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.*;

public class OI {
        public Joystick joystickLeft = new Joystick(PortMap.JOYSTICK_LEFT);
        public Joystick joystickRight = new Joystick(PortMap.JOYSTICK_RIGHT);

        // public Button intakeBalls, lowerIntakeArms, StartHopper, togglePneumatics;

        public OI() {
                // this.intakeBalls = new Button();
                // this.intakeBalls.whenHeld(new IntakeBallCommand());

                // this.lowerIntakeArms = new Button();
                // this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());

                // this.StartHopper = new Button();
                // this.StartHopper.whenPressed(new StartHopperCommand());

                // this.togglePneumatics = new Button();
                // this.togglePneumatics.toggleWhenPressed(new TogglePneumaticsCommand());

        }
}

