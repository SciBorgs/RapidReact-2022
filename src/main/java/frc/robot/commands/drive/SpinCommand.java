package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.Util;

public class SpinCommand extends CommandBase {
    private static final double EPSILON = 0.05;
    private PID headingPID;
    private double target;

    public SpinCommand(double rot) {
        this.target = Robot.localizationSubsystem.getHeading() + rot;
        this.headingPID = new PID(0.55, 0, 0.1);
    }

    @Override
    public void execute() {
        double currHeading = Robot.localizationSubsystem.getHeading();
        double diffHeading = Util.travelledAngle(currHeading, target);

        double angleOutput = this.headingPID.getOutput(diffHeading); //values negated for testing

        angleOutput = Util.normalize(angleOutput);

        Robot.driveSubsystem.spinRobot(angleOutput);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Util.travelledAngle(Robot.localizationSubsystem.getHeading(), target)) < EPSILON;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0.0, 0.0);
    }
}
