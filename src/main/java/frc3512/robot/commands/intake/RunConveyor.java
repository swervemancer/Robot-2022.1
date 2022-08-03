package frc3512.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;

public class RunConveyor extends CommandBase {

  private FrontFlywheel m_frontFlywheel;
  private BackFlywheel m_backFlywheel;
  private Intake m_intake;

  private Timer timer;

  public RunConveyor(FrontFlywheel frontFlywheel, BackFlywheel backFlywheel, Intake intake) {
    this.m_frontFlywheel = frontFlywheel;
    this.m_backFlywheel = backFlywheel;
    this.m_intake = intake;
    addRequirements(m_frontFlywheel, m_backFlywheel, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setConveyor(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopConveyor();
    m_frontFlywheel.setGoal(0.0);
    m_backFlywheel.setGoal(0.0);
    m_frontFlywheel.setGoalFromRange(false);
    m_backFlywheel.setGoalFromRange(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0);
  }
}
