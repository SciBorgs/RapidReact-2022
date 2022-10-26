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
import frc.robot.PortMap;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.EncoderSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeTest {
  IntakeSubsystem intake;
  EncoderSim encoder; // not the wpilib encoder sim

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    intake = new IntakeSubsystem();
    // intake = new IntakeSubsystem();
    encoder = new EncoderSim(PortMap.Intake.SUCK_SPARK);
  }

  // only one test is used to avoid the network table problems
  @Test
  public void runTests() {
    forwardSuckTest();
    reverseSuckTest();
  }

  private void forwardSuckTest() {
    intake.startSuck();
    assertEquals(IntakeConstants.INTAKE_SPEED, intake.getSuckSpeed(), Constants.MARGIN_OF_ERROR);
    // assertEquals(IntakeConstants.INTAKE_SPEED, encoder.getVelocity(), Constants.MARGIN_OF_ERROR);
    // ^ doesn't work because encoder.getVelocity() = 0
    // keep in mind, we have extensively tested that any properly constructed
    // EncoderSim.getVelocity()
    // will return its expected value in sim GUI
  }

  private void reverseSuckTest() {
    intake.reverseSuck();
    assertEquals(-IntakeConstants.INTAKE_SPEED, intake.getSuckSpeed(), Constants.MARGIN_OF_ERROR);
  }
}
