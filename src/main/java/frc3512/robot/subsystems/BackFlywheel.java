package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc3512.lib.controllers.FlywheelController;
import frc3512.lib.controllers.FlywheelController.FlywheelPose;
import frc3512.lib.controllers.FlywheelController.Input;
import frc3512.lib.math.InterpolatingTreeMap;
import frc3512.lib.subsystems.ControlledSubsystem;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.lib.util.NetworkTableUtil;
import frc3512.robot.Constants;

public class BackFlywheel extends ControlledSubsystem<N1, N1, N1> {

  CANSparkMax m_backGrbx = new CANSparkMax(Constants.BackFlywheel.kMotorID, MotorType.kBrushless);
  Encoder m_backEncoder =
      new Encoder(Constants.BackFlywheel.kEncoderA, Constants.BackFlywheel.kEncoderB);

  LinearSystem<N1, N1, N1> m_plant = FlywheelController.getFrontPlant();
  KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_plant,
          VecBuilder.fill(200.0),
          VecBuilder.fill(2.5),
          Constants.General.kControllerPeriodic);

  FlywheelController m_controller = new FlywheelController(FlywheelPose.Back);
  InterpolatingTreeMap<Double, Double> m_table = new InterpolatingTreeMap<>();

  Matrix<N1, N1> m_u;

  double m_range;
  boolean m_setGoalFromRange = true;

  double m_angle;
  double m_lastAngle;
  double m_time = Timer.getFPGATimestamp();
  double m_lastTime = m_time - Constants.General.kControllerPeriodic;

  double m_angularVelocity;
  LinearFilter m_velocityFilter = LinearFilter.movingAverage(4);

  // Used in test mode for manually setting flywheel goal. This is helpful for
  // measuring flywheel lookup table values.
  double m_testThrottle = 0.0;

  NetworkTableEntry m_percentageEntry =
      NetworkTableUtil.makeDoubleEntry("/Diagnostics/Front Flywheel/Percent");

  public BackFlywheel() {
    m_backGrbx.setSmartCurrentLimit(40);

    m_backGrbx.setInverted(true);

    m_backEncoder.setDistancePerPulse(FlywheelController.kDpP);
    m_backEncoder.setSamplesToAverage(5);

    CANSparkMaxUtil.setCANSparkMaxBusUsage(m_backGrbx, Usage.kMinimal);

    m_table.put(48.0, 146.225);
    m_table.put(72.0, 175.47);
    m_table.put(96.0, 204.47);
    m_table.put(120.0, 263.205);
    m_table.put(144.0, 321.695);
    m_table.put(168.0, 380.185);

    if (RobotBase.isSimulation()) {
      setGoalFromRange(false);
    }

    reset();
    setGoal(0.0);
  }

  public void setGoal(double velocity) {
    m_controller.setGoal(velocity);
  }

  public void setGoalFromRange(boolean setGoal) {
    m_setGoalFromRange = setGoal;
  }

  public void setVoltage(double voltage) {
    m_backGrbx.setVoltage(voltage);
  }

  public double throttleToReference(double throttle) {
    var remap = (1.0 - throttle) / 2.0;
    final var kLow = 100.0;
    final var kHigh = 1500.0;
    var rescale = kLow + (kHigh - kLow) * remap;
    return Math.round(rescale);
  }

  public void reset() {
    m_observer.reset();
    m_controller.reset();
    m_u.fill(0.0);
    m_angle = getAngle();
    m_lastAngle = m_angle;
  }

  public void enableFrontFlywheel() {
    enable();
  }

  public void disableFrontFlywheel() {
    disable();
  }

  public double getAngle() {
    return m_backEncoder.getDistance();
  }

  public double getAngularVelocity() {
    return m_angularVelocity;
  }

  public double getGoal() {
    return m_controller.getGoal();
  }

  public double getGoalFromRange() {
    return m_table.get(m_range);
  }

  public void stop() {
    setGoal(0.0);
  }

  public boolean atGoal() {
    return m_controller.atGoal();
  }

  public boolean isOn() {
    return getGoal() > 0.0;
  }

  public boolean isReady() {
    return isOn() && atGoal();
  }

  @Override
  public void controllerPeriodic() {

    updateDt();

    m_observer.predict(m_u, getDt());

    m_angle = getAngle();
    m_time = Timer.getFPGATimestamp();

    // WPILib uses the time between pulses in GetRate() to calculate velocity,
    // but this is very noisy for high-resolution encoders. Instead, we
    // calculate a velocity from the change in angle over change in time, which
    // is more precise.
    m_angularVelocity = m_velocityFilter.calculate((m_angle - m_lastAngle) / (m_time - m_lastTime));

    Matrix<N1, N1> y = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(getAngularVelocity());
    m_observer.correct(m_controller.getInputs(), y);
    m_u = m_controller.calculate(m_observer.getXhat());
    setVoltage(m_u.get(Input.kVoltage, 0));

    if (m_setGoalFromRange) {
      setGoal(m_table.get(m_range));
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
  }

  @Override
  public void periodic() {
    double percent = getGoal() / Constants.FrontFlywheel.kMaxAngularVelocity * 100.0;
    m_percentageEntry.setDouble(percent);
  }
}
