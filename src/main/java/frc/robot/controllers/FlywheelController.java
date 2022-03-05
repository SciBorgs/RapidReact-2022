package frc.robot.controllers;

public class FlywheelController {
    private double rpm;

    public FlywheelController(double s) {
        rpm = s;
    }

    public double getConstantRPM(int dist) {
        return rpm;
    }
    
    public double getRPMFromFunction() {
        // may not be implemented
        return 0;
    }
}
