package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AxisButton extends JoystickButton {
    private GenericHID joystick;
    private int axis;
    private double deadzone;
    private boolean backward;

    public AxisButton(GenericHID joystick, int axis) {
        this(joystick, axis, false);
    }

    public AxisButton(GenericHID joystick, int axis, boolean backward) {
        this(joystick, axis, backward, 0.1);
    }

    public AxisButton(GenericHID joystick, int axis, boolean backward, double deadzone) {
        super(joystick, 0); // We override the function that uses buttonNumber

        this.axis = axis;
        this.joystick = joystick;
        this.deadzone = deadzone;
        this.backward = backward;
    }

    @Override
    public boolean get() {
        double axisVal = joystick.getRawAxis(axis);

        return axisVal > (backward ? -deadzone : deadzone);
    }
}
