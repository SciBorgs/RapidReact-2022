import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FlywheelTest {
  FlywheelSubsystem flywheel;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    flywheel = new FlywheelSubsystem();
  }

  @Test
  public void runTests() {
    testDirection();
  }

  private void testDirection() {
    flywheel.setTargetFlywheelSpeed(10);
    assertEquals(flywheel.getTargetFlywheelSpeed(), 10);
    // assert flywheel.getCurrentFlywheelSpeed() > 0;

    flywheel.setTargetFlywheelSpeed(-10);
    assertEquals(flywheel.getTargetFlywheelSpeed(), -10);
    // assertEquals(expected, actual);
  }
}
