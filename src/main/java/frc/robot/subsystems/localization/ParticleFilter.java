package frc.robot.subsystems.localization;

import java.util.function.Supplier;

import frc.robot.subsystems.localization.models.MotionModel;
import frc.robot.subsystems.localization.models.SensorModel;
import frc.robot.util.Util;

/**
 * Implementation of a particle filter, used for determining the position of a
 * robot. This approach recursively acts on a set of 'particles', representing
 * possible states of the robot. 
 * <p>
 * One step moves each of the particles according 
 * to the control inputs or motion sensor measurements. A second step resamples
 * from the current set of particles to give more weight to particles that
 * align with sensor measurements (i.e. landmarks, beacons).
 * <p>
 * For a more detailed description of the algorithm used here, check out the 
 * below paper:
 * {@link https://www.ri.cmu.edu/pub_files/pub1/dellaert_frank_1999_2/dellaert_frank_1999_2.pdf}
 */
public class ParticleFilter {
    private double[][] particles, particlesTemp;
    private final double[] weights, cummWeights;
    private final double[] motions, sensors;
    protected final int stateDim, motionDim, sensorDim, n;

    protected MotionModel motionModel;
    protected SensorModel sensorModel;

    protected double[] flat;

    /**
     * Creates a new particle filter with a given fixed size, given the initial
     * distribution of particles.
     * @param initialStateGenerator the initial distribution of particles
     * @param motions the array to read motion measurements from
     * @param sensors the array to read sensor measurements from
     * @param numParticles the number of particles (fixed)
     */
    public ParticleFilter(Supplier<double[]> initialStateGenerator, double[] motions, double[] sensors, int numParticles) {
        this.particles = new double[numParticles][];
        this.particlesTemp = new double[numParticles][];
        for (int i = 0; i < numParticles; i++)
            this.particles[i] = initialStateGenerator.get();
        
        this.weights = new double[numParticles];
        this.cummWeights = new double[numParticles];

        Util.fill(this.weights, 1.0);
        Util.cdf(cummWeights, weights);
        this.motions = motions;
        this.sensors = sensors;

        this.stateDim = this.particles[0].length;
        this.motionDim = this.motions.length;
        this.sensorDim = this.sensors.length;
        this.n = numParticles;

        this.flat = new double[n * 2];
    }

    /**
     * Creates a new particle filter with a given fixed size, given the initial
     * distribution of particles.
     * @param initialStateGenerator the initial distribution of particles
     * @param motionDim the dimension of the motion measurement vector
     * @param sensorDim the dimension of the sensor measurement vector
     * @param numParticles the number of particles (fixed)
     */
    public ParticleFilter(Supplier<double[]> initialStateGenerator, int motionDim, int sensorDim, int numParticles) {
        this(initialStateGenerator, new double[motionDim], new double[sensorDim], numParticles);
    }

    /**
     * Sets the motion model for this particle filter. 
     * @see MotionModel
     * @param motionModel the new motion model for this particle filter
     */
    public void setMotionModel(MotionModel motionModel) { this.motionModel = motionModel; }

    /**
     * Sets the sensor model for this particle filter.
     * @see SensorModel
     * @param sensorModel the new sensor model for this particle filter
     */
    public void setSensorModel(SensorModel sensorModel) { this.sensorModel = sensorModel; }

    /**
     * Uses the provided motion model to predict the position of the robot,
     * given the current set of particles.
     */
    public void motionUpdate() {
        if (this.motionModel == null) return;
        for (int i = 0; i < n; i++) {
            particles[i] = this.motionModel.sample(particles[i], motions);
        }
    }

    /**
     * Samples from the weighted set of particles (with updated weights given by 
     * the provided sensor model) to get the updated set of particles.
     */
    public void sensorUpdate() {
        if (this.sensorModel == null) return;
        this.sensorModel.updateWeights(particles, weights, sensors);
        Util.cdf(cummWeights, weights);
        for (int i = 0; i < n; i++) {
            particlesTemp[i] = this.particles[Util.sampleDiscreteCdf(cummWeights)];
        }
        swap();
    }

    /**
     * Instead of allocating a new array for each iteration (), two arrays are
     * used, and the references particles and particlesTemp are switched for
     * convenience.
     */
    private void swap() {
        double[][] temp = particlesTemp;
        particlesTemp = particles;
        particles = temp;
    }

    /**
     * Returns the array in which sensor measurements are read from. This array
     * should be used to update sensor measurements for the particle filter.
     * @return the sensor measurement array
     */
    public double[] getSensors() { return this.sensors; }

    /**
     * Returns the array in which motion measurements are read from. This array
     * should be used to update motion measurements for the particle filter.
     * @return the motion measurement array
     */
    public double[] getMotions() { return this.motions; }

    /**
     * Returns the particles tracked by this particle filter, stored in a two
     * dimensional array.
     * @return the particles tracked by this particle filter
     */
    public double[][] getParticles() { return this.particles; }
    
    /**
     * Returns a flattened version of getParticles() in an array of length
     * numParticles * stateDim. To access values in the i-th particle with
     * dimension k, access indices k*i, k*i+1, ... , k*i+(k-1).
     * @return the particles tracked by this particle filter, in a flat array
     */
    public double[] getFlat() {
        Util.flatten(flat, particles, n, stateDim);
        return flat;
    }

    /**
     * Gets the average particle from this particle filter. 
     * <p>
     * Note: If there are many variably-sized clusters of particles, then the 
     * average particle may represent an unfeasible or unlikely state
     * (ex. the average American family has 2.5 kids.)
     * @return the average particle of this particle filter
     */
    public double[] getMeanParticle() {
        double[] meanParticle = new double[stateDim];
        for (int i = 0; i < stateDim; i++) {
            for (double[] particle : particles)
                meanParticle[i] += particle[i];
            meanParticle[i] /= n;
        }
        return meanParticle;
    }
}
