import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.*;

import edu.wpi.first.hal.HAL;
import frc.robot.PortMap;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTest {
    IntakeSubsystem intake;
    CANSparkMax motor;

    @BeforeEach
    public void setupSubsystem() {
        assert HAL.initialize(500, 0);
        intake = new IntakeSubsystem();
        motor = new CANSparkMax(PortMap.Intake.SUCK_SPARK, CANSparkMax.MotorType.kBrushless);
    }

    @AfterEach
    public void destroySubsystem() throws Exception {
        intake.close();
    }

    @DisplayName("blegh")
    @Test
    public void set_motors() {
        intake.startSuck();
        assertEquals(Constants.IntakeConstants.INTAKE_SPEED, intake.getIntakeSpeed(), 1e-2);
    }

}
