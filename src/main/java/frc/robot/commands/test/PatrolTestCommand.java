package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.FollowPathController;
import frc.robot.util.DelayedPrinter;
import frc.robot.util.Util;

public class PatrolTestCommand extends CommandBase {
    private FollowPathController pathController;
    private DelayedPrinter printer;
    private long startingTime;

    private static final double PROCEED_HEADING = Math.PI / 5;
    private static final double PROCEED_DISTANCE = 0.2;
    private static final double TIME_LIMIT = 15000;

    @Override
    public void initialize() {
        this.pathController = new FollowPathController(
            Constants.PATH_PATROL,
            PROCEED_HEADING, PROCEED_DISTANCE, true);
        this.printer = new DelayedPrinter(500);
        this.startingTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        this.pathController.move();
        printer.print("PatrolTestCommand : "
                    + "\n" + Util.indent(this.pathController.getInfoString()));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - this.startingTime > TIME_LIMIT;
    }

    @Override 
    public void end(boolean interrupted) {
        if (!interrupted)
            System.out.println("PatrolTestCommand timed out. ");
    }
}
