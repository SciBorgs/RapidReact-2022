package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controllers.PurePursuitController;
import frc.robot.util.Path;

public class PurePursuitCommand extends CommandBase {
    private PurePursuitController pp;
    private final Path path;

    public PurePursuitCommand(Path path) {
        this.path = path;
    }

    @Override
    public void initialize() {
        this.pp = new PurePursuitController(this.path, 0.201155, 0.0);
    }

    @Override
    public void execute() {
        pp.move();
    }

    @Override
    public boolean isFinished() {
        return this.pp.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }
}
