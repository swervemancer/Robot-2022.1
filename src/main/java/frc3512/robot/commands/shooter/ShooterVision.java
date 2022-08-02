package frc3512.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.BackFlywheel;
import frc3512.robot.subsystems.FrontFlywheel;

public class ShooterVision extends CommandBase {

  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;

  public ShooterVision(FrontFlywheel frontFlywheel, BackFlywheel backFlywheel) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    addRequirements(m_frontFlywheel, m_backFlywheel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_frontFlywheel.setGoalFromRange(true);
    m_backFlywheel.setGoalFromRange(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
