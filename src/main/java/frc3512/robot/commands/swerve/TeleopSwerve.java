package frc3512.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private Swerve s_Swerve;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  /** Drives around the swerve drive with a XboxController */
  public TeleopSwerve(
      Swerve s_Swerve,
      Joystick controller,
      int translationAxis,
      int strafeAxis,
      int rotationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

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

    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.General.kJoystickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.General.kJoystickDeadband) ? 0 : xAxis;
    rAxis = (Math.abs(rAxis) < Constants.General.kJoystickDeadband) ? 0 : rAxis;

    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }
}
