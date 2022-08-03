package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.intake.Intake;

public class DeployIntake extends CommandBase {

  Intake m_intake;

  /** Deploys or stows the intake fourbar */
  public DeployIntake(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    if (m_intake.isDeployed()) {
      m_intake.stow();
    } else {
      m_intake.deploy();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
