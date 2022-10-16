import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.EncoderSim;
import org.junit.jupiter.api.*;

public class IntakeTest {
  IntakeSubsystem intake;
  private EncoderSim suckEncoder;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    intake = new IntakeSubsystem();
    suckEncoder = new EncoderSim(PortMap.Intake.SUCK_SPARK);
  }

  @AfterEach
  public void shutdown() throws Exception {
    intake.close();
  }

  @Test
  public void forwardSuckTest() {
    intake.startSuck();
    System.out.println(suckEncoder.getVelocity());
    assertEquals(IntakeConstants.INTAKE_SPEED, suckEncoder.getVelocity(), Constants.MARGIN_OF_ERROR);
  }

  // @Test
  // public void reverseSuckTest() {
  //   intake.reverseSuck();
  //   assertEquals(-IntakeConstants.INTAKE_SPEED, suckEncoder.getVelocity(), Constants.MARGIN_OF_ERROR);
  // }
}
