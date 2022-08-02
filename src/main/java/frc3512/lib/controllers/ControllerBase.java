package frc3512.lib.controllers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

public abstract class ControllerBase<States extends Num, Inputs extends Num, Outputs extends Num> {

  /** Controller reference for current timestep. */
  Matrix<States, N1> m_r;

  /** Controller reference for next timestep. */
  Matrix<States, N1> m_nextR;

  /** Controller output. */
  Matrix<Inputs, N1> m_u;

  /**
   * Returns the current references.
   *
   * <p>See the State class in the derived class for what each element corresponds to.
   */
  public final Matrix<States, N1> getReferences() {
    return m_r;
  }

  /**
   * Returns the control inputs.
   *
   * <p>See the Input class in the derived class for what each element corresponds to.
   */
  public final Matrix<Inputs, N1> getInputs() {
    return m_u;
  }

  /**
   * Returns the next output of the controller.
   *
   * @param x The current state x.
   */
  public abstract Matrix<Inputs, N1> calculate(Matrix<States, N1> x);

  /**
   * Returns the next output of the controller.
   *
   * @param x The current state x.
   * @param r The next reference r.
   */
  public Matrix<Inputs, N1> calculate(Matrix<States, N1> x, Matrix<States, N1> r) {
    m_nextR = r;
    Matrix<Inputs, N1> u = calculate(x);
    m_r = m_nextR;
    return u;
  }
}
