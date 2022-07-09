package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {

  Intake m_intake;
  boolean m_ignoreSensors;

  public IntakeCargo(Intake intake, boolean ignoreSensors) {
    this.m_intake = intake;
    this.m_ignoreSensors = ignoreSensors;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.runIntake();
    m_intake.setConveyor(m_ignoreSensors);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_intake.stopConveyor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
