package frc.robot.subsystems.localization.models;

// Use for: Encoders, Pigeons
@FunctionalInterface
public interface MotionModel {
    double[] sample(double[] particle, double[] motion);
}
