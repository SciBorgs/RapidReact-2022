package frc.robot;

import frc.robot.commands.intake.*;
import frc.robot.commands.climber.ClimberExtend;
import frc.robot.commands.climber.HookMotor;
import frc.robot.commands.climber.RetractClimberArm;
import frc.robot.commands.hopper.*;
import frc.robot.commands.pneumatics.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;


public class OI {
    public Joystick leftStick, rightStick;
    public XboxController xboxController;
    public JoystickButton intakeBalls, lowerIntakeArms, StartHopper, toggleCompressor, climberArm, hookMotor, retractClimberArm;


    public OI() {
        this.leftStick = new Joystick(PortMap.JOYSTICK_LEFT);
        this.rightStick = new Joystick(PortMap.JOYSTICK_RIGHT);
        this.xboxController = new XboxController(PortMap.XBOX_CONTROLLER);

        // Intake
        this.intakeBalls = new JoystickButton(this.leftStick, PortMap.Joystick.JOYSTICK_LEFT_BUTTON);
        this.intakeBalls.whenPressed(new IntakeBallsCommand());

        this.lowerIntakeArms = new JoystickButton(this.leftStick, PortMap.Joystick.JOYSTICK_RIGHT_BUTTON);
        this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());

        // Hopper
        this.StartHopper = new JoystickButton(this.leftStick, PortMap.Joystick.JOYSTICK_TRIGGER);
        this.StartHopper.whenPressed(new StartHopperCommand());

        // Compressor
        this.toggleCompressor = new JoystickButton(this.xboxController, PortMap.XboxController.XBOX_START);
        this.toggleCompressor.toggleWhenPressed(new ToggleCompressorCommand());

        // Climber
        this.climberArm = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_CENTER_BUTTON);
        this.climberArm.whenHeld(new ClimberExtend());

        this.hookMotor = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_LEFT_BUTTON);
        this.hookMotor.whenHeld(new HookMotor().withTimeout(0.2));

        this.retractClimberArm = new JoystickButton(this.rightStick, PortMap.Joystick.JOYSTICK_TRIGGER);
        this.retractClimberArm.whenHeld(new RetractClimberArm());

    }
}
