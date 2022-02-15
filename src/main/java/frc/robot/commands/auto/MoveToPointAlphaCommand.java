package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToRingController;
import frc.robot.Constants;

public class MoveToPointAlphaCommand extends CommandBase {
    private MoveToRingController controller;

    @Override
    public void initialize() {
        this.controller = new MoveToRingController(Constants.RING_ALPHA, 0.9);
    }

    @Override
    public void execute() {
        this.controller.move();
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived();
    }
}