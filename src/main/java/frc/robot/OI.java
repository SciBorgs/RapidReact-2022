package frc.robot;

import frc.robot.commands.intake.*;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.HookMotor;
import frc.robot.commands.RetractClimberArm;
import frc.robot.commands.hopper.*;
import frc.robot.commands.pneumatics.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class OI {
        public Button intakeBalls, lowerIntakeArms, StartHopper, togglePneumatics, climberArm, hookMotor, retractClimberArm;
        public Joystick joystickLeft, joystickRight;

        public OI() {
                this.joystickLeft = new Joystick(PortMap.JOYSTICK_LEFT);
                this.joystickRight  = new Joystick(PortMap.JOYSTICK_RIGHT);
                this.intakeBalls = new Button();
                this.intakeBalls.whenPressed(new IntakeBallsCommand());
                this.lowerIntakeArms = new Button();
                this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());
                this.StartHopper = new Button();
                this.StartHopper.whenPressed(new StartHopperCommand());

                this.togglePneumatics = new Button();
                this.togglePneumatics.toggleWhenPressed(new TogglePneumaticsCommand());

                this.climberArm = new Button();
                this.climberArm.whenHeld(new ClimberExtend());

                this.hookMotor = new Button();
                this.hookMotor.whenHeld(new HookMotor().withTimeout(0.2));

                this.retractClimberArm = new Button();
                this.retractClimberArm.whenHeld(new RetractClimberArm());
                
        }
}
