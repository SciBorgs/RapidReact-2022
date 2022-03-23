package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.HoodAngleController;
import frc.robot.sciSensorsActuators.SciAbsoluteEncoder;
import frc.robot.util.PID;
import frc.robot.util.ShufflePID;

public class ShooterSubsystem extends SubsystemBase {
    private PID shooterPID;
    private ShufflePID shooterShufflePID;

    private ShuffleboardTab shooterTab;
    private NetworkTableEntry distance, hoodAngle;
    
    public CANSparkMax hood;
    //, lmotor, rmotor;
    private SciAbsoluteEncoder thruBoreEncoder;
    public ShooterSubsystem() {
        shooterPID = new PID(0.0005, 0, 0);
        shooterShufflePID = new ShufflePID("shooter", shooterPID, "big shell");

        shooterTab = Shuffleboard.getTab("shooter");
        distance = shooterTab.add("distance", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        hoodAngle = shooterTab.add("hoodangle ", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

        this.hood = new CANSparkMax(PortMap.HOOD_SPARK, MotorType.kBrushless);

        /*
        this.lmotor = new CANSparkMax(PortMap.SHOOTER_LEFT_SPARK, MotorType.kBrushless);
        this.rmotor = new C2NSparkMax(PortMap.SHOOTER_RIGHT_SPARK, MotorType.kBrushless);
        */
        
        //lmotor.follow(rmotor);
        // thruBoreEncoder = new SciAbsoluteEncoder(PortMap.HOOD_ENCODER, Constants.TOTAL_HOOD_GEAR_RATIO);
    }
    
    public final double HEIGHTDIFF = 2.08534;
    public final double CAM_MOUNT_ANGLE = 30;
    
    public double getDistance() {
        return HEIGHTDIFF/Math.tan(Math.toRadians(Robot.limelightSubsystem.getLimelightTableData("ty") + CAM_MOUNT_ANGLE));
    }

    public double getCurrentHoodAngle() {
        return thruBoreEncoder.getAngle();
    }
    public double functionAngle() {
        return HoodAngleController.getDegFromFunction(getDistance());
    }
    /*
    public void shoot(double speed) {
        rmotor.set(speed);
    }
    */

    public void moveVert(double speed) {
        if(speed > 0.1)speed = 0.1;
        if(speed <-0.1) speed = -0.1;
        hood.set(speed);

    }

    public void moveHood(double angle) {
        moveVert(-shooterPID.getOutput(angle, thruBoreEncoder.getAngle()));
    }

    public void updateGraphs() {
        distance.setDouble(getDistance());
        hoodAngle.setDouble(getCurrentHoodAngle());
    }

    public void update() {
        shooterShufflePID.update();
        updateGraphs();
    }
}
