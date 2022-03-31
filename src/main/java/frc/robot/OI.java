package frc.robot;

import frc.robot.commands.intake.*;
import frc.robot.commands.climber.ExtendClimberArm;
import frc.robot.commands.climber.ExtendHook;
import frc.robot.commands.climber.RetractClimberArm;
import frc.robot.commands.climber.RetractHook;
import frc.robot.commands.hopper.*;
import frc.robot.commands.pneumatics.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;


public class OI {
    public Joystick leftStick, rightStick;
    public XboxController xboxController;
    public JoystickButton intakeBalls, lowerIntakeArms, StartHopper, toggleCompressor, extendClimberArm, retractClimberArm, extendHook, retractHook;


    public OI() {
        this.leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        // Intake
        
        this.intakeBalls = new JoystickButton(this.leftStick, PortMap.Joystick.JOYSTICK_BUTTON_MATRIX_LEFT[1][0]);
        this.intakeBalls.whenPressed(new IntakeBallsCommand());

        this.lowerIntakeArms = new JoystickButton(this.leftStick, PortMap.Joystick.JOYSTICK_BUTTON_MATRIX_LEFT[1][1]);
        this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());

        // Hopper
        this.StartHopper = new JoystickButton(this.leftStick, PortMap.Joystick.JOYSTICK_TRIGGER);
        this.StartHopper.whenPressed(new StartHopperCommand());

        // Compressor
        this.toggleCompressor = new JoystickButton(this.xboxController, PortMap.XboxController.XBOX_START);
        this.toggleCompressor.toggleWhenPressed(new ToggleCompressorCommand());

        // Climber
        this.extendClimberArm = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_CENTER_BUTTON);
        this.extendClimberArm.whenHeld(new ExtendClimberArm());

        this.retractClimberArm = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_TRIGGER);
        this.retractClimberArm.whenHeld(new RetractClimberArm());

        this.extendHook = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_RIGHT_BUTTON);
        this.extendHook.whenHeld(new ExtendHook().withTimeout(0.2));

        this.retractHook = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_LEFT_BUTTON);
        this.retractHook.whenHeld(new RetractHook());

    }
}
