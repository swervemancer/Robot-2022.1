package frc3512.lib.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Helper functions for setting up NetworkTable entries */
public class NetworkTableUtil {

  /**
   * Creates a NetworkTable entry with a default double value of 0.
   *
   * @param name Path of network table entry.
   * @param defaultValue The entry's initial value.
   */
  public static NetworkTableEntry makeDoubleEntry(String name, double defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultDouble(defaultValue);

    return entry;
  }

  /**
   * Creates a NetworkTable entry with a default double value of 0.
   *
   * @param name Path of network table entry.
   */
  public static NetworkTableEntry makeDoubleEntry(String name) {
    return makeDoubleEntry(name, 0.0);
  }

  /**
   * Creates a NetworkTable entry with a default bool value of false.
   *
   * @param name Path of network table entry.
   * @param defaultValue The entry's initial value.
   */
  public static NetworkTableEntry makeBoolEntry(String name, boolean defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultBoolean(defaultValue);

    return entry;
  }

  /**
   * Creates a NetworkTable entry with a default bool value of false.
   *
   * @param name Path of network table entry.
   * @param defaultValue The entry's initial value.
   */
  public static NetworkTableEntry makeBoolEntry(String name) {
    return makeBoolEntry(name, false);
  }

  /**
   * Creates a NetworkTable entry with a default string value of "".
   *
   * @param name Path of network table entry.
   * @param defaultValue The entry's initial value.
   */
  public static NetworkTableEntry makeStringEntry(String name, String defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultString(defaultValue);

    return entry;
  }

  /**
   * Creates a NetworkTable entry with a default string value of "".
   *
   * @param name Path of network table entry.
   * @param defaultValue The entry's initial value.
   */
  public static NetworkTableEntry makeStringEntry(String name) {
    return makeStringEntry(name, "");
  }

  /**
   * Creates a NetworkTable entry with a default double array value of {}.
   *
   * @param name Path of network table entry.
   * @param defaultValue The entry's initial value.
   */
  public static NetworkTableEntry makeDoubleArrayEntry(String name, double[] defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultDoubleArray(defaultValue);

    return entry;
  }

  /**
   * Creates a NetworkTable entry with a default double array value of {}.
   *
   * @param name Path of network table entry.
   */
  public static NetworkTableEntry makeDoubleArrayEntry(String name) {
    double[] defaultList = {};
    return makeDoubleArrayEntry(name, defaultList);
  }
}
