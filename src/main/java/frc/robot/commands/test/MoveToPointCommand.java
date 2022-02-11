package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPoint;
import frc.robot.util.Point;

public class MoveToPointCommand extends CommandBase{
    private MoveToPoint controller;
    private final long start;

    public MoveToPointCommand() {
        super();
        start = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
        this.controller = new MoveToPoint(new Point(1, 0));
    }

    @Override
    public void execute() {
         if (controller.isFacingPoint()) { 
             if (System.currentTimeMillis() % 1000 == 0)
                 System.out.println("is facing point"); 
             controller.move(); 
         }
         else { 
            if (System.currentTimeMillis() % 1000 == 0)
                System.out.println("turning"); 
            controller.turn(); 
         }
    }

    @Override
    public boolean isFinished() {
        return 
            controller.isFacingPoint() && controller.hasArrived()
            || (System.currentTimeMillis() - start) > 20000;
    }
}
