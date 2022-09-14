package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.PortMap;

public class ClimberSubsystem implements Subsystem {
  private CANSparkMax telescope, arms;

  public ClimberSubsystem() {
    this.telescope = new CANSparkMax(PortMap.Climber.TELESCOPE_SPARK, MotorType.kBrushless);
    this.arms = new CANSparkMax(PortMap.Climber.ARMS_SPARK, MotorType.kBrushless);

    this.telescope.setIdleMode(IdleMode.kBrake);
    this.arms.setIdleMode(IdleMode.kBrake);
  }

  public void extendTelescope() {
    telescope.set(ClimberConstants.TELESCOPE_SPEED);
  }

  public void retractTelescope() {
    telescope.set(-ClimberConstants.TELESCOPE_SPEED);
  }

  public void stopTelescope() {
    telescope.set(0);
  }

  public void extendArms() {
    arms.set(ClimberConstants.ARM_SPEED);
  }

  public void retractArms() {
    arms.set(-ClimberConstants.ARM_SPEED);
  }

  public void stopArms() {
    arms.set(0);
  }
}
