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

    // Buttons | Intake + Hopper
    public JoystickButton hopper;
    
    // Buttons | Climber
    public JoystickButton extendTelescope, retractTelescope, extendArm, retractArm;

    // Buttons | Shooter
    public JoystickButton aimButton, shootButton;
    public JoystickButton lowerHoodButton, raiseHoodButton;

    public OI(boolean isXbox) {
        this.isXbox = isXbox;

        // Controllers
        this.xboxController = new XboxController(XBOX_CONTROLLER);
        this.leftStick  = new Joystick(JOYSTICK_LEFT);
        this.rightStick = new Joystick(JOYSTICK_RIGHT);

        ////////////////////
        // INITIALIZATION //
        ////////////////////

        if (isXbox) {
            // Intake
            this.intakeBalls       = new JoystickButton(this.xboxController, XBOX_B);
            this.lowerIntakeArms   = new JoystickButton(this.xboxController, XBOX_START);
            this.retractIntakeArms = new JoystickButton(this.xboxController, XBOX_BACK);

            // Intake-Hopper-Compressor
            this.hopper = new JoystickButton(this.xboxController, XBOX_A);

            // Shooter
            this.aimButton   = new JoystickButton(this.xboxController, XBOX_X);
            this.shootButton = new JoystickButton(this.xboxController, XBOX_Y);
        } else {
            // Intake
            this.intakeBalls       = new JoystickButton(this.leftStick, JOYSTICK_CENTER_BUTTON);
            this.lowerIntakeArms   = new JoystickButton(this.leftStick, JOYSTICK_RIGHT_BUTTON);
            this.retractIntakeArms = new JoystickButton(this.leftStick, JOYSTICK_LEFT_BUTTON);

            // Intake-Hopper-Compressor
            this.hopper = new JoystickButton(this.leftStick, JOYSTICK_TRIGGER);

            // Shooter
            this.aimButton   = new JoystickButton(this.rightStick, JOYSTICK_CENTER_BUTTON);
            this.shootButton = new JoystickButton(this.rightStick, JOYSTICK_TRIGGER);

        }
        
        this.lowerHoodButton = new JoystickButton(this.xboxController, XBOX_BUMPER_LEFT);
        this.raiseHoodButton = new JoystickButton(this.xboxController, XBOX_BUMPER_RIGHT);

        //////////////////////
        // COMMAND BINDINGS //
        //////////////////////

        // Climber
        this.extendTelescope  = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[0][0]);
        this.retractTelescope = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[0][1]);
        this.extendArm  = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[1][0]);
        this.retractArm = new JoystickButton(this.rightStick, JOYSTICK_BUTTON_MATRIX_RIGHT[1][1]);

    
        // Intake
        this.intakeBalls.whenHeld(new IntakeBallsCommand());
        this.lowerIntakeArms.whenPressed(new LowerIntakeArmCommand());
        this.retractIntakeArms.whenPressed(new RetractIntakeArmCommand());

        // Intake-Hopper-Compressor
        // this.startHopper.whenHeld(new StartHopperCommand());
        this.hopper.whileHeld(new StartHopperCommand());

        // Climber
        this.extendTelescope.whenHeld(new RunTelescopeCommand(false));
        this.retractTelescope.whenHeld(new RunTelescopeCommand(true));
        this.extendArm.whenHeld(new RunArmCommand(false));
        this.retractArm.whenHeld(new RunArmCommand(true));

        // Shooter
        this.aimButton.whenPressed(new AimCommandGroup());
        this.shootButton.whenPressed(new ShootSequence());

        this.lowerHoodButton.whenHeld(new LowerHoodCommand());
        this.raiseHoodButton.whenHeld(new RaiseHoodCommand());
    }
}
