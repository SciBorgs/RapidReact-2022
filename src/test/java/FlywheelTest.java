import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FlywheelTest {
  static FlywheelSubsystem flywheel = new FlywheelSubsystem();
  CANSparkMax spark;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    spark = flywheel.getSpark();
  }

  @Test
  public void testDirection() {
    // forwards
    flywheel.setTargetFlywheelSpeed(10);
    flywheel.periodic();
    assert spark.getAppliedOutput() > 0;

    // backwards
    flywheel.setTargetFlywheelSpeed(-10);
    flywheel.periodic();
    assert spark.getAppliedOutput() < 0;

    // stopping
    flywheel.setTargetFlywheelSpeed(0);
    flywheel.periodic();
    assertEquals(0, spark.get());
  }

  @Test
  public void testReasonableRange() {
    flywheel.setTargetFlywheelSpeed(2000);
    flywheel.periodic();
    assert spark.getAppliedOutput() < 50;
  }
}
