package frc3512.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc3512.lib.math.Conversions;
import frc3512.lib.swerve.REVModuleState;
import frc3512.lib.swerve.SwerveModuleConstants;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;

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

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState =
        REVModuleState.optimize(
            desiredState,
            getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which CTRE is not

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      mDriveMotor.set(percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
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
        Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio), ControlType.kPosition);
    lastAngle = angle;
  }

  private void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
    mAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfig.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    mAngleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
    mAngleEncoder = mAngleMotor.getEncoder();
    mAnglePID = mAngleMotor.getPIDController();
    mAnglePID.setP(Constants.Swerve.angleKP);
    mAnglePID.setI(Constants.Swerve.angleKI);
    mAnglePID.setD(Constants.Swerve.angleKD);
    mAnglePID.setFF(Constants.Swerve.angleKF);
    mAngleMotor.setSmartCurrentLimit(
        Constants.Swerve.angleContinuousCurrentLimit, Constants.Swerve.anglePeakCurrentLimit);
    mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    mAngleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    mDriveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kVelocityOnly);
    mDriveEncoder = mDriveMotor.getEncoder();
    mDrivePID = mDriveMotor.getPIDController();
    mDrivePID.setP(Constants.Swerve.driveKP);
    mDrivePID.setI(Constants.Swerve.driveKI);
    mDrivePID.setD(Constants.Swerve.driveKD);
    mDrivePID.setFF(Constants.Swerve.driveKF);
    mDriveMotor.setSmartCurrentLimit(
        Constants.Swerve.driveContinuousCurrentLimit, Constants.Swerve.drivePeakCurrentLimit);
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

  public SwerveModuleState getState() {
    double velocity =
        Conversions.falconToMPS(
            mDriveEncoder.getVelocity(),
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveGearRatio);
    Rotation2d angle =
        Rotation2d.fromDegrees(
            Conversions.falconToDegrees(
                mAngleEncoder.getPosition(), Constants.Swerve.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
  }
}
