package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.button.*;

public class OI {
        public Button climberArm, hookMotor, retractClimberArm;

        public OI() {
                this.climberArm = new Button();
                this.climberArm.whenHeld(new ClimberExtend());

                this.hookMotor = new Button();
                this.hookMotor.whenHeld(new HookMotor().withTimeout(0.2));

                this.retractClimberArm = new Button();
                this.retractClimberArm.whenHeld(new RetractClimberArm());

        }
}
