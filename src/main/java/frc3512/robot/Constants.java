package frc3512.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc3512.lib.swerve.SwerveModuleConstants;

/** Constants for the robot project */
public final class Constants {

  /// Joystick axis deadband range
  public static final double kJoystickDeadband = 0.05;

  /// The period at which feedback controllers run
  public static final double kControllerPeriod = 5.0;

  /** Constants revolving around joysticks * */
  public static final class Joysticks {
    // Xbox Controller port
    public static final int kXboxControllerPort = 0;

    // Appendage joystick 1 port
    public static final int kAppendageStick1Port = 1;

    // Appendage joystick 2 port
    public static final int kAppendageStick2Port = 2;
  }

  public static final class Swerve {
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.27305 * 2.0;
    public static final double wheelBase = 0.27305 * 2.0;
    public static final double wheelDiameter = 0.10033;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (5.14 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;

    /* Angle Motor PID Values */
    public static final double angleKP = 1.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.1;
    public static final double angleKF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 8.0032;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.44706;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (-0.075312 / 12);
    public static final double driveKV = (0.64857 / 12);
    public static final double driveKA = (0.044474 / 12);

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kCoast;
    public static final CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 20;
      public static final int canCoderID = 1;
      public static final double angleOffset = 101.07;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 21;
      public static final int canCoderID = 2;
      public static final double angleOffset = 140.97;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 3;
      public static final double angleOffset = 246.47;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 23;
      public static final int canCoderID = 4;
      public static final double angleOffset = 238.89;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
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
