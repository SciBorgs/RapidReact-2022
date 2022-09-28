package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.PortMap;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax telescope, arms;

  private ShuffleboardTab tab;

  public ClimberSubsystem() {
    telescope = new CANSparkMax(PortMap.Climber.TELESCOPE_SPARK, MotorType.kBrushless);
    arms = new CANSparkMax(PortMap.Climber.ARMS_SPARK, MotorType.kBrushless);

    telescope.setIdleMode(IdleMode.kBrake);
    arms.setIdleMode(IdleMode.kBrake);

    tab = Shuffleboard.getTab("Climber");
    tab.add(this);
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
