package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Extend this class and call the constructor (super(Subsystem...) in a child
 * class).
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
}
