package frc3512.lib.controllers;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc3512.robot.Constants;

public class FlywheelController extends ControllerBase<N1, N1, N1> {

  public static enum FlywheelPose {
    Front,
    Back
  }

  // Gear ratio for both flywheels
  private static double kGearRatio = 1.0 / 1.0;

  // Angle per encoder pulse
  public static double kDpP = (Math.PI * 2.0) * kGearRatio / 2048.0;

  private static double kAngularVelocityShotThreshold = 18.0;
  private static double kAngularVelocityRecoveryThreshold = 18.0;

  FlywheelPose m_pose;

  boolean m_atGoal = false;

  LinearSystem<N1, N1, N1> m_frontPlant = getFrontPlant();
  LinearQuadraticRegulator<N1, N1, N1> m_frontLQR =
      new LinearQuadraticRegulator<>(
          m_frontPlant,
          VecBuilder.fill(25.0),
          VecBuilder.fill(12.0),
          Constants.General.kControllerPeriodic);
  LinearPlantInversionFeedforward<N1, N1, N1> m_frontFF =
      new LinearPlantInversionFeedforward<>(m_frontPlant, Constants.General.kControllerPeriodic);

  LinearSystem<N1, N1, N1> m_backPlant = getBackPlant();
  LinearQuadraticRegulator<N1, N1, N1> m_backLQR =
      new LinearQuadraticRegulator<>(
          m_backPlant,
          VecBuilder.fill(25.0),
          VecBuilder.fill(12.0),
          Constants.General.kControllerPeriodic);
  LinearPlantInversionFeedforward<N1, N1, N1> m_backFF =
      new LinearPlantInversionFeedforward<>(m_backPlant, Constants.General.kControllerPeriodic);

  /** States of the flywheel system. */
  public class State {
    /// Flywheel angular velocity.
    public static final int kAngularVelocity = 0;
  }

  /** Inputs of the flywheel system. */
  public class Input {
    /// Motor voltage.
    public static final int kVoltage = 0;
  }

  /** Outputs of the flywheel system. */
  public class Output {
    /// Flywheel angular velocity.
    public static final int kAngularVelocity = 0;
  }

  public FlywheelController(FlywheelPose pose) {
    this.m_pose = pose;
    reset();
  }

  public final boolean atGoal() {
    return m_atGoal;
  }

  public double getGoal() {
    return m_nextR.get(0, 0);
  }

  public static LinearSystem<N1, N1, N1> getFrontPlant() {
    return LinearSystemId.identifyVelocitySystem(
        Constants.FrontFlywheel.kS, Constants.FrontFlywheel.kA);
  }

  public static LinearSystem<N1, N1, N1> getBackPlant() {
    return LinearSystemId.identifyVelocitySystem(
        Constants.BackFlywheel.kS, Constants.BackFlywheel.kA);
  }

  public void reset() {
    m_r.fill(0.0);
    m_nextR.fill(0.0);
  }

  public void setGoal(double angularVelocity) {
    if (m_nextR.get(0, 0) == angularVelocity) {
      return;
    }

    m_nextR.fill(angularVelocity);
    m_atGoal = false;
  }

  @Override
  public Matrix<N1, N1> calculate(Matrix<N1, N1> x) {
    if (m_nextR.get(0, 0) == 0.0) {
      m_u.fill(0.0);
    } else {
      if (m_pose == FlywheelPose.Front) {
        m_u =
            m_frontLQR
                .calculate(x, m_r)
                .plus(m_frontFF.calculate(m_nextR))
                .plus(new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Constants.FrontFlywheel.kS));
      } else if (m_pose == FlywheelPose.Back) {
        m_u =
            m_backLQR
                .calculate(x, m_r)
                .plus(m_backFF.calculate(m_nextR))
                .plus(new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Constants.BackFlywheel.kS));
      }
    }

    m_u = StateSpaceUtil.desaturateInputVector(m_u, 12.0);

    updateAtGoal(m_nextR.get(0, 0) - x.get(0, 0));
    m_r = m_nextR;

    return m_u;
  }

  private void updateAtGoal(double error) {
    if (m_atGoal && error > kAngularVelocityShotThreshold) {
      m_atGoal = false;
    } else if (!m_atGoal && error < kAngularVelocityRecoveryThreshold) {
      m_atGoal = true;
    }
  }
}
