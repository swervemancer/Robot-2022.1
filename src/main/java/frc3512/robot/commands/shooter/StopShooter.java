package frc3512.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;

public class StopShooter extends CommandBase {

  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;

  public StopShooter(FrontFlywheel frontFlywheel, BackFlywheel backFlywheel) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    addRequirements(m_frontFlywheel, m_backFlywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_frontFlywheel.setGoal(0.0);
    m_backFlywheel.setGoal(0.0);
  }
}
