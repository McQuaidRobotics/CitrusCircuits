package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Autos;

public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private Autos.AutoRoutines autoRoutine;
    private Alliance alliance = Alliance.Invalid;
    private final SendableChooser<Autos.AutoRoutines> autoRoutineChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        RobotContainer.RobotContainerInit();

        Autos.AutoRoutines[] autoRoutines = Autos.AutoRoutines.values();
        for (Autos.AutoRoutines autoRoutine : autoRoutines) {
            if (autoRoutine == Autos.AutoRoutines.NOTHING) {
                autoRoutineChooser.setDefaultOption(autoRoutine.name(), autoRoutine);
                continue;
            }
            autoRoutineChooser.addOption(autoRoutine.name(), autoRoutine);
        }
        Shuffleboard.getTab("Autos").add("Autonomous Routine", autoRoutineChooser)
            .withSize(2, 1)
            .withPosition(0, 0);
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
        var selectedRoutine = autoRoutineChooser.getSelected();
        var currAlliance = DriverStation.getAlliance();
        if (selectedRoutine != autoRoutine || alliance != currAlliance) {
            autoRoutine = selectedRoutine;
            alliance = currAlliance;
            autonomousCommand = Autos.getAutoRoutineCommand(autoRoutine);
        }
    }

    @Override
    public void autonomousInit() {
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
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
