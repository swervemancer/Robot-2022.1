package frc3512.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.BackFlywheel;
import frc3512.robot.subsystems.FrontFlywheel;

public class ShooterPreset extends CommandBase {

  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;
  private double m_frontRPS;
  private double m_backRPS;

  public ShooterPreset(
      FrontFlywheel frontFlywheel, BackFlywheel backFlywheel, double frontRPS, double backRPS) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    this.m_frontRPS = frontRPS;
    this.m_backRPS = backRPS;
    addRequirements(m_frontFlywheel, m_backFlywheel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_frontFlywheel.setGoal(m_frontRPS);
    m_backFlywheel.setGoal(m_backRPS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
