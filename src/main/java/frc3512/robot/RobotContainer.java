package frc3512.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc3512.lib.dashboard.AutonomousChooser;
import frc3512.lib.util.NetworkTableUtil;
import frc3512.robot.auto.ExampleAuto;
import frc3512.robot.commands.climbers.DeployClimbers;
import frc3512.robot.commands.climbers.RunClimbers;
import frc3512.robot.commands.intake.DeployIntake;
import frc3512.robot.commands.intake.IntakeCargo;
import frc3512.robot.commands.intake.OuttakeCargo;
import frc3512.robot.commands.intake.RunConveyor;
import frc3512.robot.commands.shooter.ShooterPreset;
import frc3512.robot.commands.shooter.ShooterVision;
import frc3512.robot.commands.shooter.ShootingVisionAim;
import frc3512.robot.commands.shooter.StopShooter;
import frc3512.robot.commands.swerve.TeleopSwerve;
import frc3512.robot.subsystems.climber.Climber;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.swerve.Swerve;
import frc3512.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Manual shooting settings
  private static final NetworkTableEntry m_manualShooting =
      NetworkTableUtil.makeBoolEntry("Settings/Manual Shooting", false);

  // Autonomous chooser
  private final AutonomousChooser m_autonChooser = new AutonomousChooser();

  // Robot subsystems
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Vision m_vision = new Vision();
  private final Swerve m_swerve = new Swerve();
  private final FrontFlywheel m_frontFlywheel = new FrontFlywheel();
  private final BackFlywheel m_backFlywheel = new BackFlywheel();

  // Joysticks + XboxController
  private final Joystick driver = new Joystick(0);
  private final Joystick m_appendageStick1 = new Joystick(Constants.Joysticks.kAppendageStick1Port);
  private final Joystick m_appendageStick2 = new Joystick(Constants.Joysticks.kAppendageStick2Port);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Climber buttons
  private final JoystickButton m_deployClimbersButton = new JoystickButton(m_appendageStick2, 1);
  private final JoystickButton m_overrideLimitsButton = new JoystickButton(m_appendageStick2, 11);

  // Intake buttons
  private final JoystickButton m_deployIntakeButton = new JoystickButton(m_appendageStick1, 1);
  private final JoystickButton m_intakeButton = new JoystickButton(m_appendageStick1, 3);
  private final JoystickButton m_outtakeButton = new JoystickButton(m_appendageStick1, 4);

  // Drive buttons
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton zeroModules =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  // General shooting buttons
  private final JoystickButton runConveyor =
      new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
  private final JoystickButton stopShooter =
      new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

  // Manual shooting buttons
  private final JoystickButton lowShot = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton highFenderShot =
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton highTarmacShot =
      new JoystickButton(driver, XboxController.Button.kY.value);

  // Vision shooting buttons
  private final JoystickButton rangedShooting =
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton visionAimShooting =
      new JoystickButton(driver, XboxController.Button.kB.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addVisionQueues();
    configureButtonBindings();
    configureAxisActions();
    addAutons();
  }

  /** Add subsystems to the vision queue */
  private void addVisionQueues() {
    m_vision.subscribeToVisionData(m_swerve.visionMeasurements);
    m_vision.subscribeToVisionData(m_frontFlywheel.visionMeasurements);
    m_vision.subscribeToVisionData(m_backFlywheel.visionMeasurements);
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

    // General Shooting Buttons
    runConveyor.whenPressed(new RunConveyor(m_frontFlywheel, m_backFlywheel, m_intake));
    stopShooter.whenPressed(new StopShooter(m_frontFlywheel, m_backFlywheel));

    if (m_manualShooting.getBoolean(false)) {
      // Manual Shooting Buttons
      lowShot.whenPressed(
          new ShooterPreset(
              m_frontFlywheel,
              m_backFlywheel,
              Constants.FrontFlywheel.kShootLow,
              Constants.BackFlywheel.kShootLow));
      highFenderShot.whenPressed(
          new ShooterPreset(
              m_frontFlywheel,
              m_backFlywheel,
              Constants.FrontFlywheel.kShootHighFender,
              Constants.BackFlywheel.kShootHighFender));
      highTarmacShot.whenPressed(
          new ShooterPreset(
              m_frontFlywheel,
              m_backFlywheel,
              Constants.FrontFlywheel.kShootHighTarmac,
              Constants.BackFlywheel.kShootHighTarmac));
    } else {
      // Vision Shooting Buttons
      rangedShooting.whenPressed(new ShooterVision(m_frontFlywheel, m_backFlywheel));
      visionAimShooting.whenPressed(
          new ShootingVisionAim(m_swerve, m_frontFlywheel, m_backFlywheel, m_intake));
    }

    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> m_swerve.zeroIMU()));
    zeroModules.whenPressed(new InstantCommand(() -> m_swerve.zeroModules()));
  }

  /** Used for joystick/xbox axis actions. */
  private void configureAxisActions() {
    m_climber.setDefaultCommand(
        new RunClimbers(
            m_climber,
            () ->
                MathUtil.applyDeadband(
                        m_appendageStick1.getRawAxis(1), Constants.General.kJoystickDeadband)
                    * 0.8,
            () ->
                MathUtil.applyDeadband(
                        m_appendageStick2.getRawAxis(1), Constants.General.kJoystickDeadband)
                    * 0.76));

    boolean fieldRelative = true;
    boolean openLoop = true;
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
  }

  /** Adds in autonomous modes */
  private void addAutons() {
    m_autonChooser.addAuton("Example", new ExampleAuto(m_swerve));
    m_autonChooser.updateAutonList();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelectedAuton();
  }
}
