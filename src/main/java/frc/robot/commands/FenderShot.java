package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.PortMap.Shooter;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FenderShot extends CommandBase {
    private ShooterSubsystem shooter;
    private HopperSubsystem hopper;
    private Debouncer isShooting;

    public FenderShot(ShooterSubsystem shooter, HopperSubsystem hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
        addRequirements(shooter, hopper);
    }

    @Override
    public void initialize() {
        shooter.setTargetFlywheelSpeed(ShooterConstants.FENDER_RPM);
        shooter.setTargetHoodAngle(ShooterConstants.FENDER_ANGLE);
        isShooting = new Debouncer(VisionConstants.TIMESCALE, DebounceType.kFalling);
    }

    @Override
    public void execute() {
        if (isShooting.calculate(shooter.atTargetAngle() && shooter.atTargetRPM())) {
            hopper.startElevator();
        } else {
            hopper.stopElevator();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetFlywheelSpeed(0);
        hopper.stopElevator();
    }
}
