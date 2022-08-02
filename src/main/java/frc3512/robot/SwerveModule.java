package frc3512.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc3512.lib.math.Conversions;
import frc3512.lib.util.OptimizedModuleState;
import frc3512.lib.util.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private CANSparkMax mAngleMotor;
  private RelativeEncoder mAngleEncoder;
  private SparkMaxPIDController mAnglePID;
  private CANSparkMax mDriveMotor;
  private RelativeEncoder mDriveEncoder;
  private SparkMaxPIDController mDrivePID;
  private CANCoder angleEncoder;
  private double lastAngle;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    // Angle Encoder
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    // Angle Motor
    mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    mAngleEncoder = mAngleMotor.getEncoder();
    mAnglePID = mAngleMotor.getPIDController();
    configAngleMotor();

    // Drive Motor
    mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    mDriveEncoder = mDriveMotor.getEncoder();
    mDrivePID = mDriveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = Constants.Swerve.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
  }

  private void configAngleMotor() {
    mAngleMotor.restoreFactoryDefaults();
    mAnglePID.setP(Constants.Swerve.angleKP);
    mAnglePID.setI(Constants.Swerve.angleKI);
    mAnglePID.setD(Constants.Swerve.angleKD);
    mAnglePID.setIZone(0.0);
    mAnglePID.setFF(Constants.Swerve.angleKF);
    mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    mAngleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    mDriveMotor.restoreFactoryDefaults();
    mDrivePID.setP(Constants.Swerve.driveKP);
    mDrivePID.setI(Constants.Swerve.driveKI);
    mDrivePID.setD(Constants.Swerve.driveKD);
    mDrivePID.setIZone(0.0);
    mDrivePID.setFF(Constants.Swerve.driveKF);
    mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
    mDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);
    mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    mDriveMotor.burnFlash();
    mDriveEncoder.setPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  private void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToNeo(
            getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
    mAngleEncoder.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    double velocity =
        Conversions.neoToMPS(
            mDriveEncoder.getVelocity(),
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveGearRatio);
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.neoToDegrees(mAngleEncoder.getPosition(), Constants.Swerve.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = OptimizedModuleState.optimize(desiredState, getState().angle);

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      mDriveMotor.set(percentOutput);
    } else {
      double velocity =
          Conversions.MPSToNeo(
              desiredState.speedMetersPerSecond,
              Constants.Swerve.wheelCircumference,
              Constants.Swerve.driveGearRatio);
      mDrivePID.setReference(
          velocity,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
    // Jittering.
    mAnglePID.setReference(
        Conversions.degreesToNeo(angle, Constants.Swerve.angleGearRatio), ControlType.kPosition, 0);
    lastAngle = angle;
  }
}
