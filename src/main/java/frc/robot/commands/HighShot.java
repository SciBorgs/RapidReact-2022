package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class HighShot extends CommandBase {
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private HopperSubsystem hopper;
    private VisionSubsystem vision;
    private Debouncer isShooting;

    public HighShot(ShooterSubsystem shooter, TurretSubsystem turret, HopperSubsystem hopper, VisionSubsystem vision) {
        this.shooter = shooter;
        this.turret = turret;
        this.hopper = hopper;
        this.vision = vision;
        addRequirements(shooter, turret, hopper, vision);
        vision.reset();
    }

    @Override
    public void initialize() {
        vision.reset();
        isShooting = new Debouncer(VisionConstants.TIMESCALE, DebounceType.kFalling);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double dist = vision.getDistance();
            shooter.setTargetHoodAngle(ShooterConstants.getHoodAngle(dist));
            shooter.setTargetFlywheelSpeed(ShooterConstants.getRPM(dist));
            turret.setTargetAngle(turret.getCurrentAngle() + vision.getXOffset());
        }

        if (isShooting.calculate(
                shooter.atTargetAngle() && shooter.atTargetRPM() && turret.atTarget() && vision.hasTarget())) {
            hopper.startElevator();
        } else {
            hopper.stopElevator();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTargetAngle(0);
        hopper.stopElevator();
        shooter.setTargetFlywheelSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return shooter.get() < 1; // ball count may or may not work, use with timeout
    }
}
