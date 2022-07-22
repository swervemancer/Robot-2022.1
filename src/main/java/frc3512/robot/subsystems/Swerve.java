package frc3512.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.NetworkTableUtil;
import frc3512.robot.Constants;

public class Swerve extends SubsystemBase {

  /** The maximum voltage that will be delivered to the drive motors. */
  public static final double MAX_VOLTAGE = 12.0;

  /** The maximum velocity of the robot in meters per second. */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      5676.0
          / 60.0
          * SdsModuleConfigurations.MK4_L4.getDriveReduction()
          * SdsModuleConfigurations.MK4_L4.getWheelDiameter()
          * Math.PI;

  /** The maximum angular velocity of the robot in radians per second. */
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(
              Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(
              Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(
              Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(
              -Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -Constants.DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, new Rotation2d());

  private final ADIS16470_IMU m_imu = new ADIS16470_IMU();

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public static final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(8, 5);

  private NetworkTableEntry m_headingEntry =
      NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Drivetrain/Heading");

  public Swerve() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule =
        Mk4SwerveModuleHelper.createNeo(
            // This parameter is optional, but will allow you to see the current state of the module
            // on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L4,
            // This is the ID of the drive motor
            Constants.DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            Constants.DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            Constants.DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is
            // facing straight forward)
            Constants.DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule =
        Mk4SwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            Constants.DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule =
        Mk4SwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            Constants.DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule =
        Mk4SwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L4,
            Constants.DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

    m_imu.calibrate();
    zeroImu();
  }

  public void zeroImu() {
    m_imu.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    return new Rotation2d(MathUtil.angleModulus(Units.degreesToRadians(-m_imu.getAngle())));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void updateOdometry() {
    m_odometry.update(
        getGyroscopeRotation(),
        getModuleState(m_frontLeftModule),
        getModuleState(m_frontRightModule),
        getModuleState(m_backLeftModule),
        getModuleState(m_backRightModule));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroscopeRotation());
  }

  private SwerveModuleState getModuleState(SwerveModule module) {
    return new SwerveModuleState(
        module.getDriveVelocity(), new Rotation2d(Units.degreesToRadians(module.getSteerAngle())));
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_headingEntry.setDouble(getGyroscopeRotation().getRadians());
  }

  public void resetModules() {
    m_frontLeftModule.set(0.0, 0.0);
    m_frontRightModule.set(0.0, 0.0);
    m_backLeftModule.set(0.0, 0.0);
    m_backRightModule.set(0.0, 0.0);
  }

  public Command followPPTrajectory(PathPlannerTrajectory trajectory) {
    PIDController xController = new PIDController(1.5, 0, 0);
    PIDController yController = new PIDController(1.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, constraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand =
        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            m_kinematics,
            xController,
            yController,
            thetaController,
            this::setModuleStates,
            this);

    return new SequentialCommandGroup(
        new InstantCommand(() -> resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> resetModules()));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
    m_frontLeftModule.set(
        desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        desiredStates[0].angle.getRadians());
    m_frontRightModule.set(
        desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        desiredStates[1].angle.getRadians());
    m_backLeftModule.set(
        desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        desiredStates[2].angle.getRadians());
    m_backRightModule.set(
        desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        desiredStates[3].angle.getRadians());
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroscopeRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    if (Math.abs(xSpeed) < 0.1 && Math.abs(ySpeed) < 0.1 && Math.abs(rot) < 0.1) {
      m_frontLeftModule.set(
          states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          m_frontLeftModule.getSteerAngle());
      m_frontRightModule.set(
          states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          m_frontRightModule.getSteerAngle());
      m_backLeftModule.set(
          states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          m_backLeftModule.getSteerAngle());
      m_backRightModule.set(
          states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          m_backRightModule.getSteerAngle());
    } else {
      m_frontLeftModule.set(
          states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[0].angle.getRadians());
      m_frontRightModule.set(
          states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[1].angle.getRadians());
      m_backLeftModule.set(
          states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[2].angle.getRadians());
      m_backRightModule.set(
          states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[3].angle.getRadians());
    }
  }
}
