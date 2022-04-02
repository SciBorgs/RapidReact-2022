package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.sciSensorsActuators.SciEncoder;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.subsystems.localization.models.MotionModel;
import frc.robot.subsystems.localization.models.SensorModel;
import frc.robot.util.Point;
import frc.robot.util.Util;
import frc.robot.autoProfile.AutoProfile;
import frc.robot.Constants;

public class LocalizationSubsystem extends SubsystemBase {
    // robot state
    private static final double[] POS_INIT_STD_DEV = {0.1, 0.1};
    private Point pos;
    private double prevDistance, prevHeading;

    // sensors
    public SciEncoder totalEncoder;
    public SciPigeon pigeon;
    public DigitalInput ballSwitch;

    // localization models
    public static final int NUM_PARTICLES = 500;
    public ParticleFilter particleFilter;           // [2] Robot X, Robot Y

    public MotionModel encoderPigeonModel;          // [2] Encoder (ds) + Pigeon (theta)
    public double[] motionValues;

    public SensorModel limelightLimitSwitchModel;   // [2] Limelight (r) + Intake Limit Switch
    public double[] sensorValues;

    public LocalizationSubsystem() {
        this.pos = AutoProfile.STARTING_POINT;

        this.totalEncoder = new SciEncoder(
            Constants.WHEEL_ENCODER_GEAR_RATIO, Constants.WHEEL_CIRCUMFERENCE,
            Robot.driveSubsystem.lFront.getEncoder(),
            Robot.driveSubsystem.lMiddle.getEncoder(),
            Robot.driveSubsystem.lBack.getEncoder(),

            Robot.driveSubsystem.rFront.getEncoder(),
            Robot.driveSubsystem.rMiddle.getEncoder(),
            Robot.driveSubsystem.rBack.getEncoder()
        );

        this.totalEncoder.setDistance(0);
        this.totalEncoder.setInverted(false, false, false, true, true, true);

        this.pigeon = new SciPigeon(PortMap.PIGEON_ID);
        this.pigeon.setAngle(AutoProfile.STARTING_HEADING);

        this.ballSwitch = new DigitalInput(PortMap.INTAKE_SWITCH);

        // create particle filter

        this.motionValues = new double[2];
        this.sensorValues = new double[2];

        this.particleFilter = new ParticleFilter(
            Util.gaussianPairSampler(POS_INIT_STD_DEV, this.pos.toArray()),
            this.motionValues, this.sensorValues, NUM_PARTICLES);

        this.encoderPigeonModel = new MotionModel() {
            @Override
            public double[] sample(double[] particle, double[] motion) {
                double dx = motion[0] * Math.cos(motion[1]);
                double dy = motion[0] * Math.sin(motion[1]);
                return Util.sampleGaussioid(
                    new double[] {dx, dy},
                    new double[] {particle[0] + dx, particle[1] + dy}
                );
            }
        };

        this.limelightLimitSwitchModel = new SensorModel() {
            @Override
            public double weight(double[] particle, double[] sensors) {
                double weight = 1.0;

                double limelight = sensors[0];
                if (limelight >= 0.0)
                    weight /= (0.5 + limelight - Util.distance(particle, Constants.POINT_HUB));

                double limitSwitch = sensors[1];
                if (limitSwitch == 1.0)
                    weight /= (0.5 + Math.min(Util.distance(particle, Constants.RED_BALLS),
                                              Util.distance(particle, Constants.BLUE_BALLS)));

                return weight;
            }
        };

        this.particleFilter.setMotionModel(this.encoderPigeonModel);
        this.particleFilter.setSensorModel(this.limelightLimitSwitchModel);
    }

    public double[] get()      { return new double[] {this.pos.x, this.pos.y, this.pigeon.getAngle()}; }
    public Point  getPos()     { return this.pos; }
    public double getX()       { return this.pos.x; }
    public double getY()       { return this.pos.y; }
    public double getVel()     { return this.totalEncoder.getSpeed(); }
    public double getHeading() { return this.prevHeading; }
    public double getBackwardsHeading() { return this.prevHeading + Math.PI; }

    public void update() {
        motionUpdate();
        sensorUpdate();
        this.pos = new Point(this.particleFilter.getMeanParticle());
    }

    public void motionUpdate() {
        double currDistance = totalEncoder.getDistance();
        double diffDistance = currDistance - prevDistance;
        double currHeading = this.pigeon.getAngle();
        double avgHeading = (currHeading + this.prevHeading) / 2;

        this.motionValues[0] = diffDistance;
        this.motionValues[1] = avgHeading;
        this.particleFilter.motionUpdate();

        this.prevHeading = currHeading;
        this.prevDistance = currDistance;
    }

    public void sensorUpdate() {
        this.sensorValues[0] = Robot.shooterSubsystem.getDistance();
        this.sensorValues[1] = this.ballSwitch.get() ? 1.0 : 0.0;
    }

    public void reset() {
        this.pos = AutoProfile.STARTING_POINT;
        this.pigeon.setAngle(AutoProfile.STARTING_HEADING);
        this.prevHeading = this.pigeon.getAngle();
    }
}
