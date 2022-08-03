package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.intake.Intake;

public class OuttakeCargo extends CommandBase {

  Intake m_intake;

  public OuttakeCargo(Intake intake) {
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.runOuttake();
    m_intake.setConveyorOuttake();
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
