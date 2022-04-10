package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.PID;

public class SpinCommand extends CommandBase {
    private PID headingPID = new PID(0.55, 0, 0);;

    
}
