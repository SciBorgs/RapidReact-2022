package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.pneumatics.*;
import frc.robot.commands.shooter.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.*;

import static frc.robot.PortMap.*;
import static frc.robot.PortMap.Joystick.*;
import static frc.robot.PortMap.XboxController.*;

public class OI {
    public boolean isXbox;

    // Controllers
    public Joystick leftStick, rightStick;
    public XboxController xboxController;

    // Buttons | Intake
    public JoystickButton intakeBalls, lowerIntakeArms, retractIntakeArms;
    
    // Buttons | Hopper-Pneumatics
    public JoystickButton startHopper, toggleCompressor;

    // Buttons | Intake + Hopper
    public JoystickButton intakeHopperGroup;
    
    // Buttons | Climber
    public JoystickButton extendTelescope, retractTelescope, extendArm, retractArm;

    // Buttons | Shooter
    public JoystickButton aimButton, shootButton;

    public OI(boolean isXbox) {
        this.isXbox = isXbox;

        // Controllers
        if (isXbox) {
            this.xboxController = new XboxController(XBOX_CONTROLLER);
        } else {
            this.leftStick  = new Joystick(JOYSTICK_LEFT);
            this.rightStick = new Joystick(JOYSTICK_RIGHT);
        }

        ////////////////////
        // INITIALIZATION //
        ////////////////////

        if (isXbox) {
            // Intake
            this.intakeBalls       = new JoystickButton(this.xboxController, XBOX_B);
            this.lowerIntakeArms   = new JoystickButton(this.xboxController, XBOX_Y);
            this.retractIntakeArms = new JoystickButton(this.xboxController, XBOX_BACK);

            // Intake-Hopper-Compressor
            this.startHopper       = new JoystickButton(this.xboxController, XBOX_STICK_LEFT_BUTTON);
            this.intakeHopperGroup = new JoystickButton(this.xboxController, XBOX_STICK_RIGHT_BUTTON);
            this.toggleCompressor  = new JoystickButton(this.xboxController, XBOX_START);

            // Climber
            this.extendTelescope  = new JoystickButton(this.xboxController, XBOX_TRIGGER_LEFT);
            this.retractTelescope = new JoystickButton(this.xboxController, XBOX_TRIGGER_RIGHT);
            this.extendArm  = new JoystickButton(this.xboxController, XBOX_BUMPER_LEFT);
            this.retractArm = new JoystickButton(this.xboxController, XBOX_BUMPER_RIGHT);

            // Shooter
            this.aimButton   = new JoystickButton(this.xboxController, XBOX_A);
            this.shootButton = new JoystickButton(this.xboxController, XBOX_X);
        } else {
            // Intake
            this.intakeBalls       = new JoystickButton(this.leftStick, JOYSTICK_LEFT_BUTTON);
            this.lowerIntakeArms   = new JoystickButton(this.leftStick, JOYSTICK_RIGHT_BUTTON);
            this.retractIntakeArms = new JoystickButton(this.leftStick, JOYSTICK_CENTER_BUTTON);

            // Intake-Hopper-Compressor
            // this.startHopper       = new JoystickButton(this.leftStick, JOYSTICK_TRIGGER);
            this.intakeHopperGroup = new JoystickButton(this.leftStick, JOYSTICK_LEFT_BUTTON);
            this.toggleCompressor  = new JoystickButton(this.leftStick, JOYSTICK_RIGHT_BUTTON);

            // Climber
            this.extendTelescope  = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[0][0]);
            this.retractTelescope = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[0][1]);
            this.extendArm  = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[1][0]);
            this.retractArm = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[1][1]);

            // Shooter
            this.aimButton   = new JoystickButton(this.rightStick, JOYSTICK_CENTER_BUTTON);
            this.shootButton = new JoystickButton(this.rightStick, JOYSTICK_TRIGGER);
        }

        //////////////////////
        // COMMAND BINDINGS //
        //////////////////////

        // Intake
        this.intakeBalls.whenPressed(new IntakeBallsCommand().withTimeout(5));
        this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());
        this.retractIntakeArms.whenPressed(new RetractIntakeArmCommand());

        // Intake-Hopper-Compressor
        // this.startHopper.whenHeld(new StartHopperCommand());
        this.intakeHopperGroup.whileHeld(new ParallelDeadlineGroup(new IntakeBallsCommand().withTimeout(5), new StartHopperCommand()));
        this.toggleCompressor.toggleWhenPressed(new ToggleCompressorCommand());

        // Climber
        this.extendTelescope.whenHeld(new RunTelescopeCommand(false));
        this.retractTelescope.whenHeld(new RunTelescopeCommand(true));
        this.extendArm.whenHeld(new RunArmCommand(false));
        this.retractArm.whenHeld(new RunArmCommand(true));

        // Shooter
        this.aimButton.whenPressed(new AimCommandGroup());
        this.shootButton.whenPressed(new ShootSequence());
    }
}
