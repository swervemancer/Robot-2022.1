package frc3512.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.swerve.Swerve;

public class TurnToAngle extends CommandBase {

  private final Swerve m_swerve;
  private ProfiledPIDController m_pid;
  TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(6.0, 3.3);
  private double m_goal;

  public TurnToAngle(Swerve swerve, double goal) {
    this.m_swerve = swerve;
    this.m_goal = goal;
    m_pid = new ProfiledPIDController(0.6, 0.0, 12.0, m_constraints);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.reset(m_swerve.getYaw().getDegrees());
    m_pid.setGoal(m_goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed = m_pid.calculate(m_swerve.getYaw().getDegrees(), 0);
    m_swerve.drive(new Translation2d(), -rotationSpeed, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(new Translation2d(), 0.0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pid.atGoal();
  }
}
