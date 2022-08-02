package frc3512.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot.commands.intake.RunConveyor;
import frc3512.robot.commands.swerve.VisionAim;
import frc3512.robot.subsystems.BackFlywheel;
import frc3512.robot.subsystems.FrontFlywheel;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.Swerve;

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
