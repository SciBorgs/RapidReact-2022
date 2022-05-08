package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DPadButton extends JoystickButton {
    private final GenericHID joystick;
    private final int povIndex;
    private final Direction direction;

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        public final int angle;

        private Direction(int angle) {
            this.angle = angle;
        }
    }

    public DPadButton(GenericHID joystick, Direction direction) {
        this(joystick, 0, direction);
    }

    public DPadButton(GenericHID joystick, int povIndex, Direction direction) {
        super(joystick, 0); // We override the function that uses buttonNumber

        this.povIndex = povIndex;
        this.joystick = joystick;
        this.direction = direction;
    }

    
    
    @Override
    public boolean get() {
        int dpadPosition = joystick.getPOV(this.povIndex);
        return (dpadPosition == this.direction.angle) || (dpadPosition == (this.direction.angle + 45) % 360)
                || (dpadPosition == (this.direction.angle + 315) % 360);
    }
}
