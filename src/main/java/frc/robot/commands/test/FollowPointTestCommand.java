package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controllers.FollowPointController;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.Util;

public class FollowPointTestCommand extends CommandBase{
    private FollowPointController controller;
    private DelayedPrinter printer;

    private static final double DISTANCE_TOLERANCE = 0.05;

    public FollowPointTestCommand() {
        super();
        this.printer = new DelayedPrinter(500);
    }

    @Override
    public void initialize() {
        this.controller = new FollowPointController(DISTANCE_TOLERANCE);
    }

    @Override
    public void execute() {
        this.controller.move(Constants.POINT_TEST);
        printer.print("FollowPointTestCommand : "
                    + "\n" + Util.indent(this.controller.getInfoString()));
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived(Constants.POINT_TEST);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Stopped");
    }
}
