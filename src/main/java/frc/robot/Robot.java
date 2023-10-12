package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Autos;

public class Robot extends TimedRobot {

    private Command autonomousCommand = Autos.TEST_PATH;
    // private final SendableChooser<Autos.AutoRoutines> autoRoutineChooser = new
    // SendableChooser<>();

    @Override
    public void robotInit() {
        RobotContainer.RobotContainerInit();

        // Autos.AutoRoutines[] autoRoutines = Autos.AutoRoutines.values();
        // for (Autos.AutoRoutines autoRoutine : autoRoutines) {
        // if (autoRoutine == AutoRoutines.NOTHING) {
        // autoRoutineChooser.setDefaultOption(autoRoutine.name(), autoRoutine);
        // continue;
        // }
        // autoRoutineChooser.addOption(autoRoutine.name(), autoRoutine);
        // }
        // robotContainer.driverTab.add("Autonomous Routine", autoRoutineChooser)
        // .withSize(2, 1)
        // .withPosition(0, 0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // m_autonomousCommand =
        // m_robotContainer.getAutonomousCommand(autoRoutineChooser.getSelected());

        // if (m_autonomousCommand != null) {
        // m_autonomousCommand.schedule();
        // }
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
