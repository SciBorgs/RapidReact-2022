package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;

import org.photonvision.targeting.PhotonPipelineResult;

public class Following {
    // public static final double K_TX = 1.;
    public static final double K_TA = 1.;
    
    public static final double TX_P = 0.025;
    public static final double TX_D = TX_P/5;
    public static final double TA_P = 0.6;
    public static final double TA_WEIGHT = 1./10.;
    public static PID txPID, taPID;
    public static double taAvr;

    public static double tx;
    public static double ta;

    static {
        txPID = new PID(TX_P, 0, TX_D);
        taPID = new PID(TA_P, 0, 0);
    }

    public static void follow() {
        PhotonPipelineResult result = Robot.photonVisionSubsystem.getResult();
    
        if (result.hasTargets()) {
            ta = Robot.photonVisionSubsystem.getTarget().getArea();
            tx = -Robot.photonVisionSubsystem.getTarget().getYaw();  
            taAvr = TA_WEIGHT * ta + (1 - TA_WEIGHT) * taAvr;
            double distance = 10 / (taAvr * taAvr);
            double forward = -taPID.getOutput(Math.exp(-0.1 * taAvr), 0);
            // double forward = -Math.exp(0.5 * taPID.getOutput(distance, 0));

            System.out.println("Distance: " + distance);
            System.out.println();
            System.out.println("TX: " + tx);
            // System.out.println("Forward: " + forward * 100);
            // System.out.println("tx: " + tx);
            // System.out.println("taAvg: " + taAvr);
            Robot.driveSubsystem.setSpeedForwardAngle(forward, txPID.getOutput(tx, 0));
        } else {
            Robot.driveSubsystem.setSpeed(0, 0);
        }

    }
}
