package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ShooterData;

public class AimHoodCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;
    private double[][] hoodData;

    public AimHoodCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.hoodData = ShooterData.getHoodAngleData("src/main/java/frc/robot/controllers/placeHolder.csv");
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = vision.getDistance();
        double angle = ShooterData.calcHoodAngle(hoodData, distance);
        shooter.setTargetHoodAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return shooter.isAtTarget();
    }
}
