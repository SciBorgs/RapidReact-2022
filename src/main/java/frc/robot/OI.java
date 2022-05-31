package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.DPadButton;

import static frc.robot.PortMap.*;

public class OI {
    public boolean isXbox;

    // Controllers
    public final Joystick leftStick, rightStick;
    public final XboxController xboxController;

    // Final so we don't forget to map a button.

    // Buttons | Intake
    public final JoystickButton intakeBalls, toggleIntake;

    // Buttons | Hopper-Pneumatics
    public final JoystickButton startHopper, toggleCompressor;

    // Buttons | Intake + Hopper
    public final JoystickButton intakeHopperGroup;

    // Buttons | Climber
    public final JoystickButton extendTelescope, retractTelescope, extendArm, retractArm;

    // Buttons | Shooter
    public final JoystickButton shootButton;

    public OI(boolean isXbox) {
        this.isXbox = isXbox;

        // Unconditionally initialize the input devices. Not being present is fine, and
        // it'll make sure we don't forget to init them (ie. the previous behavior was
        // to only init Xbox if we were using Xbox, but we need joysticks to drive).

        this.leftStick = new Joystick(InputDevices.JOYSTICK_LEFT);
        this.rightStick = new Joystick(InputDevices.JOYSTICK_RIGHT);

        this.xboxController = new XboxController(InputDevices.XBOX_CONTROLLER);

        if (isXbox) {
            // Intake
            this.intakeBalls = new JoystickButton(this.xboxController, XboxControllerMap.Button.B);
            this.toggleIntake = new JoystickButton(this.xboxController, XboxControllerMap.Button.Y);

            // Intake-Hopper-Compressor
            this.startHopper = new JoystickButton(this.xboxController, XboxControllerMap.Button.STICK_LEFT);
            this.intakeHopperGroup = new JoystickButton(this.xboxController, XboxControllerMap.Button.STICK_RIGHT);
            this.toggleCompressor = new JoystickButton(this.xboxController, XboxControllerMap.Button.START);

            // Climber
            this.extendTelescope = new DPadButton(this.xboxController, DPadButton.Direction.UP);
            this.retractTelescope = new DPadButton(this.xboxController, DPadButton.Direction.DOWN);
            // TODO: make sure extend and retract are actually forward and backward relative
            // to the center of the robot
            // (we probably mixed them up)
            this.extendArm = new DPadButton(this.xboxController, DPadButton.Direction.LEFT);
            this.retractArm = new DPadButton(this.xboxController, DPadButton.Direction.RIGHT);

            // Shooter
            this.shootButton = new JoystickButton(this.xboxController, XboxControllerMap.Button.X);
        } else {
            // Intake
            this.intakeBalls = new JoystickButton(this.leftStick, JoystickMap.Button.LEFT);
            this.toggleIntake = new JoystickButton(this.leftStick, JoystickMap.Button.RIGHT);

            // Intake-Hopper-Compressor
            this.startHopper = new JoystickButton(this.leftStick, JoystickMap.Button.TRIGGER);
            this.intakeHopperGroup = new JoystickButton(this.leftStick, JoystickMap.Button.CENTER);
            this.toggleCompressor = new JoystickButton(this.rightStick, JoystickMap.Button.CENTER);

            // Climber
            this.extendTelescope = new DPadButton(this.rightStick, DPadButton.Direction.UP);
            this.retractTelescope = new DPadButton(this.rightStick, DPadButton.Direction.DOWN);
            this.extendArm = new DPadButton(this.rightStick, DPadButton.Direction.LEFT);
            this.retractArm = new DPadButton(this.rightStick, DPadButton.Direction.RIGHT);

            // Shooterl
            this.shootButton = new JoystickButton(this.rightStick, JoystickMap.Button.TRIGGER);
        }
    }
}