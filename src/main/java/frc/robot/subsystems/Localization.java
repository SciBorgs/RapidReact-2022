package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.PortMap;
import frc.robot.sciSensorsActuators.SciPigeon;
import frc.robot.util.Point;

public class Localization extends SubsystemBase {
    private Point location;
    private final int STARTING_X = 0, STARTING_Y = 0;

    public Encoder leftEncoder, rightEncoder;
    public SciPigeon pigeon;

    public Localization() {
        location = new Point(STARTING_X, STARTING_Y);
        pigeon = new SciPigeon(PortMap.PIGEON_ID);

        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(0, 1);
    }

    public double getX() { return location.getX(); }
    public double getY() { return location.getY(); }

    public void updateLocation(double x, double y) {
        location.setX(x);
        location.setY(y);
    }

}