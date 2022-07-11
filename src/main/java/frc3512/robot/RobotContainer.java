package frc3512.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc3512.robot.commands.climbers.DeployClimbers;
import frc3512.robot.commands.climbers.RunClimbers;
import frc3512.robot.commands.intake.DeployIntake;
import frc3512.robot.commands.intake.IntakeCargo;
import frc3512.robot.commands.intake.OuttakeCargo;
import frc3512.robot.subsystems.Climber;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Robot subsystems
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Vision m_vision = new Vision();

  // Joysticks + XboxController
  private final XboxController m_controller =
      new XboxController(Constants.Joysticks.kXboxControllerPort);
  private final Joystick m_appendageStick1 = new Joystick(Constants.Joysticks.kAppendageStick1Port);
  private final Joystick m_appendageStick2 = new Joystick(Constants.Joysticks.kAppendageStick2Port);

  // Joystick + XboxController buttons
  private final JoystickButton m_deployClimbersButton = new JoystickButton(m_appendageStick2, 1);
  private final JoystickButton m_overrideLimitsButton = new JoystickButton(m_appendageStick2, 11);
  private final JoystickButton m_deployIntakeButton = new JoystickButton(m_appendageStick1, 1);
  private final JoystickButton m_intakeButton = new JoystickButton(m_appendageStick1, 3);
  private final JoystickButton m_outtakeButton = new JoystickButton(m_appendageStick1, 4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    configureAxisActions();
    addAutons();
  }

  /** Used for defining button actions. */
  private void configureButtonBindings() {
    // Climber buttons
    m_deployClimbersButton.whenPressed(new DeployClimbers(m_climber));
    m_overrideLimitsButton.whenPressed(new InstantCommand(m_climber::overrideLimits, m_climber));

    // Intake buttons
    m_deployIntakeButton.whenPressed(new DeployIntake(m_intake));
    m_intakeButton.whenHeld(new IntakeCargo(m_intake, false));
    m_outtakeButton.whenHeld(new OuttakeCargo(m_intake));
  }

  /** Used for joystick/xbox axis actions. */
  private void configureAxisActions() {
    m_climber.setDefaultCommand(
        new RunClimbers(
            m_climber,
            () -> MathUtil.applyDeadband(m_appendageStick1.getRawAxis(1), 0.1) * 0.8,
            () -> MathUtil.applyDeadband(m_appendageStick2.getRawAxis(1), 0.1) * 0.76));
  }

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
