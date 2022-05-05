import frc.robot.util.Util;

import java.util.Set;

import org.junit.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BlockingTest {
    @Test
    private void blockTest() {
        BlockingSubsystem dummy = new BlockingSubsystem();
        Command block = Util.blockSubsystems(Set.of(dummy));
        Command shouldNotBeScheduled = new Command() {
            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(dummy);
            }
        };
        CommandScheduler.getInstance().schedule(shouldNotBeScheduled);
        assert(dummy.getCurrentCommand() == block);
    }
}
