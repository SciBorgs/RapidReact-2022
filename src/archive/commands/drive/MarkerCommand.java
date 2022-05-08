package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class MarkerCommand extends InstantCommand {
    private String message;

    public MarkerCommand(String message) {
        this.message = message;
    }

    @Override
    public void execute() {
        System.out.println(message);
    }
}
