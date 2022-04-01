package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.pneumatics.*;
import frc.robot.commands.shooter.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

import static frc.robot.PortMap.*;
import static frc.robot.PortMap.Joystick.*;
import static frc.robot.PortMap.XboxController.*;

public class OI {
    // Controllers
    public Joystick leftStick, rightStick;
    public XboxController xboxController;

    // Buttons | Intake
    public JoystickButton intakeBalls, lowerIntakeArms;
    
    // Buttons | Hopper-Pneumatics
    public JoystickButton startHopper, toggleCompressor;
    
    // Buttons | Climber
    public JoystickButton extendTelescope, retractTelescope, extendArm, retractArm;

    // Buttons | Shooter
    public JoystickButton aimButton, shootButton;

    public OI() {
        // Controllers
        this.leftStick  = new Joystick(JOYSTICK_LEFT);
        this.rightStick = new Joystick(JOYSTICK_RIGHT);
        this.xboxController = new XboxController(XBOX_CONTROLLER);

        // Intake
        this.intakeBalls = new JoystickButton(this.leftStick, JOYSTICK_BUTTON_MATRIX_LEFT[1][0]);
        this.intakeBalls.whenPressed(new IntakeBallsCommand());

        this.lowerIntakeArms = new JoystickButton(this.leftStick, JOYSTICK_BUTTON_MATRIX_LEFT[1][1]);
        this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());

        // Hopper
        this.startHopper = new JoystickButton(this.leftStick, JOYSTICK_TRIGGER);
        this.startHopper.whenPressed(new StartHopperCommand());

        // Compressor
        this.toggleCompressor = new JoystickButton(this.xboxController, XBOX_START);
        this.toggleCompressor.toggleWhenPressed(new ToggleCompressorCommand());

        // Climber
        this.extendTelescope = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[0][0]);
        this.extendTelescope.whenHeld(new RunTelescopeCommand(false));

        this.retractTelescope = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[1][0]);
        this.retractTelescope.whenHeld(new RunTelescopeCommand(true));

        this.extendArm = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[0][1]);
        this.extendArm.whenHeld(new RunArmCommand(false));

        this.retractArm = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[1][1]);
        this.retractArm.whenHeld(new RunArmCommand(true));

        // Shooter
        this.aimButton = new JoystickButton(this.rightStick, JOYSTICK_CENTER_BUTTON);
        this.aimButton.whenPressed(new AimCommandGroup());

        this.shootButton = new JoystickButton(this.rightStick, JOYSTICK_TRIGGER);
        this.shootButton.whenPressed(new ShootCommandGroup());
    }
}
