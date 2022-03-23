package frc.robot;

import frc.robot.commands.intake.*;
import frc.robot.commands.hopper.*
import frc.robot.commands.pneumatics.*

import edu.wpi.first.wpilibj2.command.button.*;

public class OI {
        public Button intakeBalls, lowerIntakeArms, StartHopper, togglePneumatics;

        public OI() {
               this.intakeBalls = new Button();
               this.intakeBalls.whenPressed(new IntakeBallsCommand());

               this.lowerIntakeArms = new Button();
               this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());

               this.StartHopper = new Button();
               this.StartHopper.whenPressed(new StartHopperCommand());

               this.togglePneumatics = new Button();
                this.togglePneumatics.toggleWhenPressed(new TogglePneumaticsCommand()); */

        }
}

