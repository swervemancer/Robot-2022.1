package frc3512.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;

public class ResetModules extends CommandBase {

  private Swerve m_swerve;

  public ResetModules(Swerve subsystem) {
    this.m_swerve = subsystem;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {
    m_swerve.resetModules();
  }
}
