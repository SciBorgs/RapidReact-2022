import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.*;

public class IntakeTest {
  IntakeSubsystem intake;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    intake = new IntakeSubsystem();
  }

  @AfterEach
  public void shutdown() throws Exception {
    intake.close();
  }

  @Test
  public void forwardSuckTest() {
    intake.startSuck();
    assertEquals(IntakeConstants.INTAKE_SPEED, intake.getSuckSpeed(), Constants.MARGIN_OF_ERROR);
  }

  @Test
  public void reverseSuckTest() {
    intake.reverseSuck();
    assertEquals(-IntakeConstants.INTAKE_SPEED, intake.getSuckSpeed(), Constants.MARGIN_OF_ERROR);
  }
}
