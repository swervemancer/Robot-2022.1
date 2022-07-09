package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.lib.util.NetworkTableUtil;
import frc3512.robot.Constants;

public class Climber extends SubsystemBase {

  public final int kSwitchConstant = 3000;

  public enum ClimberState {
    kIdle,
    kReachedHeightLimit,
    kReadyToClimb,
    kRetracted
  }

  AnalogInput m_leftClimberSwitch = new AnalogInput(Constants.Climber.kLeftMagneticSwitch);
  AnalogInput m_rightClimberSwitch = new AnalogInput(Constants.Climber.kRightMagenticSwitch);

  CANSparkMax m_leftGrbx = new CANSparkMax(Constants.Climber.kLeftClimberID, MotorType.kBrushless);
  CANSparkMax m_rightGrbx =
      new CANSparkMax(Constants.Climber.kRightClimberID, MotorType.kBrushless);

  Solenoid m_solenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.kClimberSolenoidChannel);

  boolean m_ignoreLimits = false;

  NetworkTableEntry m_leftTopSwitchEntry =
      NetworkTableUtil.MakeBoolEntry("/Diagnostics/Climber/Left At Switch");
  NetworkTableEntry m_rightTopSwitchEntry =
      NetworkTableUtil.MakeBoolEntry("/Diagnostics/Climber/Right At Switch");

  /** Subsystem class for the climber */
  public Climber() {
    CANSparkMaxUtil.SetCANSparkMaxBusUsage(m_leftGrbx, Usage.kPositionOnly);
    m_leftGrbx.setSmartCurrentLimit(40);
    CANSparkMaxUtil.SetCANSparkMaxBusUsage(m_rightGrbx, Usage.kPositionOnly);
    m_rightGrbx.setSmartCurrentLimit(40);
  }

  public void setClimbers(double leftSpeed, double rightSpeed) {
    if (m_ignoreLimits) {
      m_leftGrbx.set(leftSpeed);
      m_rightGrbx.set(rightSpeed);
    } else {
      if (!hasLeftPassedTopLimit() || leftSpeed > 0) {
        m_leftGrbx.set(leftSpeed);
      } else {
        m_leftGrbx.set(0.0);
      }

      if (!hasRightPassedTopLimit() || rightSpeed > 0) {
        m_rightGrbx.set(rightSpeed);
      } else {
        m_rightGrbx.set(0.0);
      }
    }
  }

  public void deployClimbers() {
    m_solenoid.set(true);
  }

  public void stowClimbers() {
    m_solenoid.set(false);
  }

  public boolean areDeployed() {
    return m_solenoid.get();
  }

  public void overrideLimits() {
    m_ignoreLimits = true;
  }

  public boolean hasRightPassedTopLimit() {
    return (m_rightClimberSwitch.getValue() < kSwitchConstant);
  }

  public boolean hasLeftPassedTopLimit() {
    return (m_leftClimberSwitch.getValue() < kSwitchConstant);
  }

  @Override
  public void periodic() {
    m_leftTopSwitchEntry.setBoolean(hasLeftPassedTopLimit());
    m_rightTopSwitchEntry.setBoolean(hasRightPassedTopLimit());
  }
}
