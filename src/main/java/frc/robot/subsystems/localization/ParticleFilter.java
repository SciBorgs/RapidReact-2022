package frc.robot.subsystems.localization;

import java.util.function.Supplier;

import frc.robot.subsystems.localization.models.MotionModel;
import frc.robot.subsystems.localization.models.SensorModel;
import frc.robot.util.Util;

/**
 * Particle filter.
 * https://www.ri.cmu.edu/pub_files/pub1/dellaert_frank_1999_2/dellaert_frank_1999_2.pdf
 */
public class ParticleFilter {
    private double[][] particles, particlesTemp;
    private final double[] weights, cummWeights;
    private final double[] motions, sensors;
    protected final int stateDim, motionDim, sensorDim, n;

    protected MotionModel motionModel;
    protected SensorModel sensorModel;

    protected double[] flat;

    public ParticleFilter(Supplier<double[]> initialStateGenerator, double[] motions, double[] sensors, int numParticles) {
        this.particles = new double[numParticles][];
        this.particlesTemp = new double[numParticles][];
        for (int i = 0; i < numParticles; i++)
            this.particles[i] = initialStateGenerator.get();
        
        this.weights = new double[numParticles];
        this.cummWeights = new double[numParticles];

        Util.fill(this.weights, 1.0 / numParticles);
        Util.cdf(cummWeights, weights);
        this.motions = motions;
        this.sensors = sensors;

        this.stateDim = this.particles[0].length;
        this.motionDim = this.motions.length;
        this.sensorDim = this.sensors.length;
        this.n = numParticles;

        this.flat = new double[n * 2];
    }

    public void setMotionModel(MotionModel motionModel) { this.motionModel = motionModel; }
    public void setSensorModel(SensorModel sensorModel) { this.sensorModel = sensorModel; }

    public void motionUpdate() {
        if (this.motionModel == null) return;
        for (int i = 0; i < n; i++) {
            particles[i] = this.motionModel.sample(particles[i], motions);
        }
    }

    public void sensorUpdate() {
        if (this.sensorModel == null) return;
        this.sensorModel.updateWeights(particles, weights, sensors);
        Util.cdf(cummWeights, weights);
        for (int i = 0; i < n; i++) {
            particlesTemp[i] = this.particles[Util.sampleDiscreteCdf(cummWeights)];
        }
        swap();
    }

    private void swap() {
        double[][] temp = particlesTemp;
        particlesTemp = particles;
        particles = temp;
    }

    public double[] getSensors() { return this.sensors; }
    public double[] getMotions() { return this.motions; }
    public double[][] getParticles() { return this.particles; }
    
    public double[] getFlat() {
        Util.flatten(flat, particles, n, stateDim);
        return flat;
    }

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
