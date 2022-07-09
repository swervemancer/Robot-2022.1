// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc3512.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Joysticks + XboxController
  private final XboxController m_controller =
      new XboxController(Constants.Joysticks.kXboxControllerPort);
  private final Joystick m_appendageStick1 = new Joystick(Constants.Joysticks.kAppendageStick1Port);
  private final Joystick m_appendageStick2 = new Joystick(Constants.Joysticks.kAppendageStick2Port);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    configureAxisActions();
    addAutons();
  }

  /** Used for defining button actions. */
  private void configureButtonBindings() {}

  /** Used for joystick/xbox axis actions. */
  private void configureAxisActions() {}

  /** Adds in autonomous modes */
  private void addAutons() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
