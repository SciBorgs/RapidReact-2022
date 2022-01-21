package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax;

//import static frc.robot.Subsystems.intake

public class LowerIntakeArm implements Command {

    private final double INTAKE_SPEED = 0.9;

    @Override
    public void initialize() {
        Robot.intake.setArmSpeed(this.INTAKE_SPEED);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        Robot.intake.setArmSpeed(0);
        Robot.intake.setIdle(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
