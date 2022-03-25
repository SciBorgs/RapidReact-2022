package frc.robot.subsystems.localization.models;

/**
 * Represents the model used by a particle filter to predict the position of a
 * particle given certain control inputs or motion measurements. A model defines
 * the probability density of the state space (possible positions) given the
 * previous state and control inputs / measurements. 
 * <p>
 * Some examples of sensors that may be used for a motion model are an encoder
 * (for determining distances) and a gyro (for determining orientation).
 * <p>
 * The probability density
 * itself is not required for the localization algorithm - instead the model
 * will specify a method for <i>sampling from this distribution</i>.
 * <p>
 * Due to the flawed and inexact nature of sensors and their measurements, it is
 * advised that the sample() method incorporate noise (e.x. through a gaussian).
 * <p>
 * For example, the following code predicts the state of a 1D robot, given the
 * previous state and the displacement from the previous state, incorporating
 * noise proportional to the displacement.
 * <pre>
 * // sampleGaussian() samples from a gaussian distribution at the mean and stdev=1
 *  double[] sample(double[] particle, double[] motion) {
 *      double prevState = particle[0];
 *      double d = motion[0];
 *      return new double[] {prevState + d + sampleGaussian() * d};
 *  }
 * </pre>
 * or using lambda notation:
 * <pre>
 * particleFilter.setMotionModel((particle, motion) -> {
 *      double prevState = particle[0];
 *      double d = motion[0];
 *      return new double[] {prevState + d + sampleGaussian() * d};
 *  });
 * </pre>
 */
@FunctionalInterface
public interface MotionModel {
    /**
     * Samples from the probability density defined by this model.
     * @see MotionModel
     * @param particle the previous state of the particle
     * @param motion the motion measurements to incorporate
     * @return a particle sampled from the distribution
     */
    double[] sample(double[] particle, double[] motion);
}
