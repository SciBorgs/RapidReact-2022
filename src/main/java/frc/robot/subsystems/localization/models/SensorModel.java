package frc.robot.subsystems.localization.models;

// Use for: Intake limit switch, Limelight (Hub)
@FunctionalInterface
public interface SensorModel {
    double weight(double[] particle, double[] sensors);

    default void updateWeights(double[][] particles, double[] weights, double[] sensors) {
        for (int i = 0; i < particles.length; i++) {
            weights[i] = weight(particles[i], sensors);
        }
    }
}
