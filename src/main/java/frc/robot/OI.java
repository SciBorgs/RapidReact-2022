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
    public final JoystickButton runIntake, reverseIntake, actuateIntake;

    // Buttons | Hopper-Pneumatics
    public final JoystickButton runHopper, reverseHopper, toggleCompressor;

    // Buttons | Climber
    public final JoystickButton extendTelescope, retractTelescope, extendArm, retractArm;

    // Buttons | Shooter
    public final JoystickButton hopperShoot;
    public final JoystickButton runFlywheel;

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
            this.runIntake = new JoystickButton(this.xboxController, XboxControllerMap.Button.X);
            this.reverseIntake = new JoystickButton(this.xboxController, 2); // TODO: UPDATE WITH PROPER PORT
            this.actuateIntake = new JoystickButton(this.xboxController, XboxControllerMap.Button.BACK);
 
            // Intake-Hopper-Compressor
            this.runHopper = new JoystickButton(this.xboxController, XboxControllerMap.Button.A);
            this.reverseHopper = new JoystickButton(this.xboxController, 3); // TODO: UPDATE WITH PROPER PORT
            this.toggleCompressor = new JoystickButton(this.xboxController, XboxControllerMap.Button.START);

            // Climber
            this.extendTelescope = new JoystickButton(this.xboxController, XboxControllerMap.Button.Y);
            this.retractTelescope = new JoystickButton(this.xboxController, XboxControllerMap.Button.B);
            // TODO: make sure extend and retract are actually forward and backward relative
            // to the center of the robot
            // (we probably mixed them up)
            this.extendArm = new DPadButton(this.xboxController, DPadButton.Direction.LEFT);
            this.retractArm = new DPadButton(this.xboxController, DPadButton.Direction.RIGHT);

            // Shooter
            this.hopperShoot = new JoystickButton(this.xboxController, XboxControllerMap.Button.BUMPER_RIGHT);
            this.runFlywheel = new JoystickButton(this.xboxController, XboxControllerMap.Button.BUMPER_LEFT);
        } else {
            // Intake
            this.runIntake = new JoystickButton(this.leftStick, JoystickMap.Button.CENTER);
            this.reverseIntake = new JoystickButton(this.leftStick, 4); // TODO: UPDATE WITH PROPER PORT
            this.actuateIntake = new JoystickButton(this.leftStick, JoystickMap.Button.RIGHT);

            // Intake-Hopper-Compressor
            this.runHopper = new JoystickButton(this.rightStick, JoystickMap.Button.CENTER);
            this.reverseHopper = new JoystickButton(this.rightStick, JoystickMap.Button.CENTER); //TODO: UPDATE WITH PROPER PORT
            this.toggleCompressor = new JoystickButton(this.rightStick, JoystickMap.Button.CENTER);

            // Climber
            this.extendTelescope = new DPadButton(this.rightStick, DPadButton.Direction.UP);
            this.retractTelescope = new DPadButton(this.rightStick, DPadButton.Direction.DOWN);
            this.extendArm = new DPadButton(this.rightStick, DPadButton.Direction.LEFT);
            this.retractArm = new DPadButton(this.rightStick, DPadButton.Direction.RIGHT);

            // Shooter
            this.hopperShoot = new JoystickButton(this.rightStick, JoystickMap.Button.TRIGGER);
            this.runFlywheel = new JoystickButton(this.leftStick, JoystickMap.Button.TRIGGER); // TODO possibly change
        }
    }
}