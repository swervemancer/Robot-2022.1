package frc3512.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.HashMap;
import java.util.Map;

public class AutonomousChooser {

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private Map<String, Command> m_autonList = new HashMap<String, Command>();

  public AutonomousChooser() {
    m_chooser.setDefaultOption("No-op", new InstantCommand());

    SmartDashboard.putData(m_chooser);
  }

  public void addAuton(String name, Command auton) {
    m_autonList.put(name, auton);
  }

  public void updateAutonList() {
    for (Map.Entry<String, Command> auton : m_autonList.entrySet()) {
      String currName = (String) auton.getKey();
      Command currAuton = (Command) auton.getValue();

      m_chooser.addOption(currName, currAuton);
    }
  }

  public Command getSelectedAuton() {
    return m_chooser.getSelected();
  }
}
