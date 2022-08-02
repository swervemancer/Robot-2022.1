package frc3512.lib.subsystems;

import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;

public abstract class ControlledSubsystem<
        States extends Num, Inputs extends Num, Outputs extends Num>
    extends SubsystemBase {

  private static double m_lastTime = Timer.getFPGATimestamp();
  private static double m_nowBegin = Timer.getFPGATimestamp();
  private static double m_dt = Constants.General.kControllerPeriodic;
  boolean m_isEnabled = true;

  public void enable() {
    m_lastTime = Timer.getFPGATimestamp() - Constants.General.kControllerPeriodic;
    m_isEnabled = true;
  }

  public void disable() {
    m_isEnabled = false;
  }

  public boolean isEnabled() {
    return m_isEnabled;
  }

  public double getDt() {
    return m_dt;
  }

  public abstract void controllerPeriodic();

  public void updateDt() {
    m_nowBegin = Timer.getFPGATimestamp();
    m_dt = m_nowBegin - m_lastTime;

    if (m_dt == 0.0) {
      m_dt = Constants.General.kControllerPeriodic;
      System.err.printf("ERROR @ t = %d: dt = 0\n", m_nowBegin);
    }

    if (m_dt > 10.0) {
      m_dt = Constants.General.kControllerPeriodic;
    }
  }

  @Override
  public void periodic() {}
}
