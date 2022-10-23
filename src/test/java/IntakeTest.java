// THIS IS STUPID
// CONTEXT:
// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/unit-testing.html
// However,
// https://github.com/wpilibsuite/allwpilib/issues/3953
// and
// simulating encoders doesn't work because SimEncoder.getVelocity() returns 0.0 in unit tests
// despite returning reasonable values in sim GUI
// as of now, resetting will not happen (sorry siggy)

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeTest {
  IntakeSubsystem intake;
  // EncoderSim encoder;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    intake = new IntakeSubsystem();
    // intake = new IntakeSubsystem();
    // encoder = new EncoderSim(PortMap.Intake.SUCK_SPARK);
  }

  @Test
  public void runTests() {
    forwardSuckTest();
    reverseSuckTest();
  }

  private void forwardSuckTest() {
    intake.startSuck();
    assertEquals(IntakeConstants.INTAKE_SPEED, intake.getSuckSpeed(), Constants.MARGIN_OF_ERROR);
  }

  private void reverseSuckTest() {
    intake.reverseSuck();
    assertEquals(-IntakeConstants.INTAKE_SPEED, intake.getSuckSpeed(), Constants.MARGIN_OF_ERROR);
  }
}
