package frc3512.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;

public class DriveSwerve extends CommandBase {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private Swerve m_Swerve;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  public DriveSwerve(
      Swerve swerve,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.m_Swerve = swerve;
    addRequirements(m_Swerve);

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
  }

  @Override
  public void execute() {
    double yAxis = -controller.getRawAxis(translationAxis);
    double xAxis = -controller.getRawAxis(strafeAxis);
    double rAxis = -controller.getRawAxis(rotationAxis);

    yAxis = (Math.abs(yAxis) < Constants.kJoystickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.kJoystickDeadband) ? 0 : xAxis;
    rAxis = (Math.abs(rAxis) < Constants.kJoystickDeadband) ? 0 : rAxis;

    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    m_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }
}
