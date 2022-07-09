package frc3512.robot;

/** Constants for the robot project */
public final class Constants {

  /** Constants revolving around joysticks * */
  public static final class Joysticks {
    // Xbox Controller port
    public static final int kXboxControllerPort = 0;

    // Appendage joystick 1 port
    public static final int kAppendageStick1Port = 1;

    // Appendage joystick 2 port
    public static final int kAppendageStick2Port = 2;
  }

  /** Constants for the intake subsystem * */
  public static final class Intake {
    /// Arm motor CAN ID
    public static final int kArmMotorID = 50;

    /// Mini arm motor CAN ID
    public static final int kMiniArmMotorID = 52;

    /// Arm solenoid channel
    public static final int kArmChannel = 4;

    /// Conveyor motor CAN ID
    public static final int kConveyorMotorID = 51;

    /// Lower proximity sensor digial input channel
    public static final int kLowerSensorChannel = 3;

    /// Upper proximity sensor digital input channel
    public static final int kUpperSensorChannel = 2;
  }

  /** Constants for the climber subsystem * */
  public static final class Climber {
    public static final int kLeftClimberID = 40;
    public static final int kRightClimberID = 41;

    public static final int kLeftMagneticSwitch = 0;
    public static final int kRightMagenticSwitch = 2;

    public static final int kClimberSolenoidChannel = 5;
  }
}
