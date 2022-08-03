package frc3512.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot.commands.intake.RunConveyor;
import frc3512.robot.commands.swerve.VisionAim;
import frc3512.robot.subsystems.flywheels.BackFlywheel;
import frc3512.robot.subsystems.flywheels.FrontFlywheel;
import frc3512.robot.subsystems.intake.Intake;
import frc3512.robot.subsystems.swerve.Swerve;

public class ShootingVisionAim extends SequentialCommandGroup {

  public ShootingVisionAim(
      Swerve swerve, FrontFlywheel frontFlywheel, BackFlywheel backFlywheel, Intake intake) {
    addCommands(
        new VisionAim(swerve),
        new ParallelCommandGroup(
            new ShooterVision(frontFlywheel, backFlywheel),
            new RunConveyor(frontFlywheel, backFlywheel, intake)));
  }
}
