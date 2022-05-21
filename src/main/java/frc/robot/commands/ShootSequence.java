package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BallCounter;

public class ShootSequence extends CommandBase {
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private HopperSubsystem hopper;
    private VisionSubsystem vision;
    private BallCounter count;

    public ShootSequence(ShooterSubsystem shooter, TurretSubsystem turret, HopperSubsystem hopper, VisionSubsystem vision, BallCounter count) {
        this.shooter = shooter;
        this.turret = turret;
        this.hopper = hopper;
        this.vision = vision;
        this.count = count;
        addRequirements(shooter, turret, hopper, vision);
    }

    @Override
    public void initialize() {
        vision.reset();
    }

    @Override
    public void execute() {
        if (!vision.hasTarget())
            return;

        double dist = vision.getDistance();
        shooter.setTargetHoodAngle(ShooterConstants.getHoodAngle(dist));
        shooter.setTargetFlywheelSpeed(ShooterConstants.getRPM(dist));
        turret.setTargetAngle(turret.getCurrentAngle() + vision.getXOffset());
        
        if (shooter.atTargetAngle() && shooter.atTargetRPM() && turret.atTarget()) {
            hopper.startElevator();
        } else {
            hopper.stopElevator(); // TODO might cause serious issues during testing. if it does, add a Debouncer or some other kind of delay
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTargetAngle(0);
        shooter.setTargetFlywheelSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return count.get() < 1; // ball count may or may not work, use with timeout
    }
}