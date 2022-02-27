package frc.robot.controllers;

import frc.robot.subsystems.NetworkTableSubsystem;

/**
 * General interface for movement controllers.
 * @param <T> TargetType the type of the target
 * @param <T> State an enum representing the state of the controller
 */
public interface MovementController<TargetType, State extends Enum<State>> {
    TargetType getTarget();
    TargetType getCurrentValue();
    void setTarget(TargetType target);
    boolean atTarget();

    State getCurrentState();
    boolean isFinished();

    void move();
    void stop();
    
    void setBindings(NetworkTableSubsystem ntsubsystem);
    void resetPIDs();
}
