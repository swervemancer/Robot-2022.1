package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Intake extends SubsystemBase {

  Timer conveyorTimer;

  CANSparkMax m_conveyorMotor =
      new CANSparkMax(Constants.Intake.kConveyorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax m_intakeMotor =
      new CANSparkMax(Constants.Intake.kArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax m_miniArmMotor =
      new CANSparkMax(Constants.Intake.kMiniArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

  DigitalInput m_upperSensor = new DigitalInput(Constants.Intake.kUpperSensorChannel);
  DigitalInput m_lowerSensor = new DigitalInput(Constants.Intake.kLowerSensorChannel);

  Solenoid m_fourbar = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.kArmChannel);

  /** Subsystem class for the intake */
  public Intake() {
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_miniArmMotor, Usage.kMinimal);
    m_miniArmMotor.setSmartCurrentLimit(80);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_intakeMotor, Usage.kMinimal);
    m_intakeMotor.setSmartCurrentLimit(80);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_conveyorMotor, Usage.kMinimal);
    m_conveyorMotor.setSmartCurrentLimit(80);
  }

  public void deploy() {
    m_fourbar.set(true);
  }

  public void stow() {
    m_fourbar.set(false);
  }

  public boolean isDeployed() {
    return m_fourbar.get();
  }

  public void runIntake() {
    m_intakeMotor.set(0.8);
    m_miniArmMotor.set(-0.8);
  }

  public void runOuttake() {
    m_intakeMotor.set(-0.8);
    m_miniArmMotor.set(0.8);
  }

  public void stopIntake() {
    m_intakeMotor.set(0.0);
    m_miniArmMotor.set(0.0);
  }

  public void setConveyor(boolean ignoreSensors) {
    if (ignoreSensors) {
      m_conveyorMotor.set(0.45);
    } else {
      m_conveyorMotor.set(0.8);
    }
  }

  public void setConveyorOuttake() {
    m_conveyorMotor.set(-0.8);
  }

  public void stopConveyor() {
    m_conveyorMotor.set(0.0);
  }

  public boolean isConveyorRunning() {
    return m_conveyorMotor.get() > 0.0;
  }

  public boolean isUpperSensorBlocked() {
    return !m_upperSensor.get();
  }

  public boolean isLowerSensorBlocked() {
    return !m_lowerSensor.get();
  }

  public void resetTimer() {
    conveyorTimer.reset();
    conveyorTimer.start();
  }

  @Override
  public void periodic() {}
}
