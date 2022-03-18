package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoProfile;
import frc.robot.Robot;
import frc.robot.controllers.PurePursuitController;

public class PurePursuitTestCommand extends CommandBase {
    private PurePursuitController pp;

    @Override
    public void initialize() {
        this.pp = new PurePursuitController(AutoProfile.PATH_TEST, 0.201155, 0.0);
    }

    @Override
    public void execute() {
        Robot.networkTableSubsystem.createControllerBindings("pp", "pp", pp, p -> p.toArray(), new double[] {0, 0});
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
