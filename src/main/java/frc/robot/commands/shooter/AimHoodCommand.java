package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AimHoodCommand extends CommandBase {
    private ShooterSubsystem shooter;
    public static final double ANGLE = 15;
    // private double[][] hoodData;

    public AimHoodCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        // hoodData = ShooterData.getHoodAngleData("src/main/java/frc/robot/controllers/placeHolder.csv");
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setNextRHood(shooter.getCurrentHoodAngle());
        shooter.setTargetHoodAngle(ANGLE);

    }

    @Override
    public boolean isFinished(){
        return shooter.isAtTarget();
    }
}
