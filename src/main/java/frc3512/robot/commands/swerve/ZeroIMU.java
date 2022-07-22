package frc3512.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;

public class ZeroIMU extends CommandBase {

  private Swerve m_swerve;

  public ZeroIMU(Swerve subsystem) {
    this.m_swerve = subsystem;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {
    m_swerve.zeroImu();
  }
}
