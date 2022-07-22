package frc3512.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DriveSwerve extends CommandBase {

  private final Swerve drivetrainSubsystem;
  private final DoubleSupplier xSpeed, ySpeed, turningSpeed;
  private final SlewRateLimiter xSpeedLimiter, ySpeedLimiter, rotLimiter;

  public DriveSwerve(
      Swerve swerveSubsystem, DoubleSupplier xSpd, DoubleSupplier ySpd, DoubleSupplier turningSpd) {
    this.drivetrainSubsystem = swerveSubsystem;
    this.xSpeed = xSpd;
    this.ySpeed = ySpd;
    this.turningSpeed = turningSpd;
    this.xSpeedLimiter = new SlewRateLimiter(3);
    this.ySpeedLimiter = new SlewRateLimiter(3);
    this.rotLimiter = new SlewRateLimiter(3);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speedX =
        -xSpeedLimiter.calculate(MathUtil.applyDeadband(xSpeed.getAsDouble(), 0.1))
            * Swerve.MAX_VELOCITY_METERS_PER_SECOND
            * 0.5;

    double speedY =
        -ySpeedLimiter.calculate(MathUtil.applyDeadband(ySpeed.getAsDouble(), 0.1))
            * Swerve.MAX_VELOCITY_METERS_PER_SECOND
            * 0.5;

    double rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(turningSpeed.getAsDouble(), 0.02))
            * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * 0.5;

    drivetrainSubsystem.drive(speedX, speedY, rot, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(0.0, 0.0, 0.0, true);
  }
}
