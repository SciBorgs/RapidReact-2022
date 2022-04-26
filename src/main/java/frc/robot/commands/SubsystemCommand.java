package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Extend this class and call the constructor (super(Subsystem...) in a child
 * class).
 * <p>
 * SubsystemCommands may be scheduled safely using the Util.schedule() method.
 * If the command must be part of a command group, you may wrap this command
 * inline using the wrap() method.
 */
public abstract class SubsystemCommand extends CommandBase {
    protected boolean hasNullReferences;

    public SubsystemCommand(Subsystem... subsystems) {
        this.hasNullReferences = false;
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) hasNullReferences = true;
        }
    }

    public boolean hasNullReferences() {
        return this.hasNullReferences;
    }

    public Command wrap() {
        SubsystemCommand wrapped = this;
        return new CommandBase() {
            public void execute() {
                if (!wrapped.hasNullReferences) wrapped.execute();
            }

            public void end(boolean interrupted) {
                if (!wrapped.hasNullReferences) wrapped.end(interrupted);
            }

            public boolean isFinished() {
                return wrapped.isFinished() || wrapped.hasNullReferences;
            }
        };
    }
}
