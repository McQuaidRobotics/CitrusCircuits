package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.Autos.AutoRoutines;
import frc.robot.util.CTREConfigs;
import frc.robot.util.NTpreferences;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final SendableChooser<Autos.AutoRoutines> autoRoutineChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    NTpreferences.loadPreferences();
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

    Autos.AutoRoutines[] autoRoutines = Autos.AutoRoutines.values();
    for (Autos.AutoRoutines autoRoutine : autoRoutines) {
      if (autoRoutine == AutoRoutines.NOTHING) {
        autoRoutineChooser.setDefaultOption(autoRoutine.name(), autoRoutine);
        continue;
      }
      autoRoutineChooser.addOption(autoRoutine.name(), autoRoutine);
    }
    m_robotContainer.driverTab.add("Autonomous Routine", autoRoutineChooser)
      .withSize(2, 1)
      .withPosition(2, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoRoutineChooser.getSelected());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
