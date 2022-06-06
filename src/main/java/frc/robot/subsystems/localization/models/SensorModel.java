package frc.robot.subsystems.localization.models;

/**
 * Represents the model used by a particle filter to determing the probability
 * of a state given sensor inputs. A model defines the probability density of 
 * the state space (possible measurements) given the current state. 
 * <p>
 * The probability density
 * itself is not required for the localization algorithm - instead the model
 * will specify a method for <i>sampling from this distribution</i>.
 * <p>
 * Due to the flawed and inexact nature of sensors and their measurements, it is
 * advised that the sample() method incorporate noise (e.x. through a gaussian).
 * <p>
 * For example, the following code represents a model dictating that the
 * displacement of a 1D robot must be close to a multiple of 10.
 * <pre>
 *  double weight(double[] particle, double[] sensors) {
 *      double x = particle[0];
 *      double diff = Math.abs(x) % 10;
 *      return Math.exp(-diff*diff); // gaussian-esque curve
 *  }
 * </pre>
 * or using lambda notation:
 * <pre>
 * particleFilter.setSensorModel((particle, motion) -> 
 *      Math.exp(-Math.pow(Math.abs(particle[0]) % 10), 2);
 *  );
 * </pre>
 */
@FunctionalInterface
public interface SensorModel {
    /**
     * Weights the particle according to this model.
     * @see SensorModel
     * @param particle the particle
     * @param motion the sensor measurements to incorporate
     * @return the resulting weight of the particle
     */
    double weight(double[] particle, double[] sensors);

    /**
     * Updates the weights stored in an array, using this sensor model and the
     * sensor measurements.
     * @param particles the current particles
     * @param weights the array to update
     * @param sensors the array containing the sensor measurements.
     */
    default void updateWeights(double[][] particles, double[] weights, double[] sensors) {
        for (int i = 0; i < particles.length; i++) {
            weights[i] = weight(particles[i], sensors);
        }
    }
}
