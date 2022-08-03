package frc3512.robot.commands.climbers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.climber.Climber;

import java.util.function.DoubleSupplier;

public class RunClimbers extends CommandBase {

  Climber m_climber;
  DoubleSupplier m_leftSpeed, m_rightSpeed;

  public RunClimbers(Climber climber, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.m_climber = climber;
    this.m_leftSpeed = leftSpeed;
    this.m_rightSpeed = rightSpeed;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climber.setClimbers(m_leftSpeed.getAsDouble(), m_rightSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.setClimbers(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
