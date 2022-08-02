package frc3512.lib.math;

public class Conversions {

  /**
   * @param counts Neo Counts
   * @param gearRatio Gear Ratio between Neo and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double neoToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / gearRatio);
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Neo and Mechanism
   * @return Neo Counts
   */
  public static double degreesToNeo(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / gearRatio);
    return ticks;
  }

  /**
   * @param rpm RPM read from the Spark Max (Velocity configured to use RPM by default.)
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
   * @return Neo Velocity Counts
   */
  public static double neoToMPS(double rpm, double circumference, double gearRatio) {
    double wheelMPS = (rpm * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
   * @return Neo Velocity Counts (Velocity is configured to use RPM by default.)
   */
  public static double MPSToNeo(double mps, double circumference, double gearRatio) {
    double wheelRPM = ((mps * 60) / circumference);
    return wheelRPM;
  }
}
