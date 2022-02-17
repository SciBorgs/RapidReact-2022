package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.SpinController;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.Util;
import frc.robot.Constants;
import frc.robot.Robot;

public class SpinTestCommand extends CommandBase {
    private SpinController spinController;
    private DelayedPrinter printer;

    private static final double HEADING_TOLERANCE = Math.PI / 8;

    @Override
    public void initialize() {
        this.spinController = new SpinController(HEADING_TOLERANCE);
        this.printer = new DelayedPrinter(500);
    }

    @Override
    public void execute() {
        this.spinController.facePoint(Constants.POINT_HUB);
        // this.spinController.faceAwayFromPoint(Constants.POINT_HUB);
        printer.print("SpinTestCommand : "
                    + "\n" + Util.indent(this.spinController.getInfoString()));
    }

    @Override
    public boolean isFinished() {
        return this.spinController.facingPoint(Constants.POINT_HUB);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Stopped");
        Robot.driveSubsystem.setSpeed(0, 0);
    }
}
