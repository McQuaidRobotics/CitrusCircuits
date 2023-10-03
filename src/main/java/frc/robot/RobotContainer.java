package frc.robot;

import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.commands.superstructure.OperatorPrefs.PickupMode;
import frc.robot.commands.superstructure.OperatorPrefs.ScoreLevel;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Swerve swerve = new Swerve();
    private final SuperStructure superStructure = new SuperStructure();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        configureDriverBindings();
        configureOperatorBindings();
        driverShuffleboard();
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driveController.getRawAxis(translationAxis),
                        () -> -driveController.getRawAxis(strafeAxis),
                        () -> -driveController.getRawAxis(rotationAxis))
        );
    }

    private void configureDriverBindings() {
        // Bumpers/Triggers
        driveController.rightBumper().onTrue(new SuperstructureCommands.TransitionToPlace(superStructure));
        driveController.leftBumper().onTrue(new SuperstructureCommands.TransitionToPickup(superStructure));
        driveController.leftTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STOW));
        driveController.rightTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STANDBY));

        // used for testing
        driveController.pov(0).onTrue(new InstantCommand(() -> superStructure.runEndEffector(12.0, false), superStructure));
        driveController.pov(180).onTrue(new InstantCommand(() -> superStructure.runEndEffector(-12.0, false), superStructure));
        driveController.pov(90).onTrue(new InstantCommand(() -> superStructure.runEndEffector(0.0, false), superStructure));
        driveController.x().onTrue(new StateManager.CmdTransitionState(superStructure, States.HOME));
        driveController.a().onTrue(new StateManager.CmdTransitionState(superStructure, States.PLACE_LOW));
        driveController.b().onTrue(new StateManager.CmdTransitionState(superStructure, States.PLACE_MID));
        driveController.y().onTrue(new StateManager.CmdTransitionState(superStructure, States.PLACE_HIGH));

        // Center Buttons
        driveController.start().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    }

    // [driver]
    // RightBumber: pickup position 
    // LeftBumper: place position
    // RightTrigger: placing standby
    // LeftTrigger: home/stow
    // Start: zero gyro
    
    // [operator]
    // Y: set desired score level to high
    // B: set desired score level to mid
    // A: set desired score level to low
    // X: home/stow
    // RightBumper: set desired pickup method to ground
    // LeftBumper: set desired pickup method to shelf
    // RightTrigger: set desired gamepiece to cube
    // LeftTrigger: set desired gamepiece to cone


    private void configureOperatorBindings() {
        // Face buttons
        operatorController.y().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.HIGH)));
        operatorController.b().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.MID)));
        operatorController.a().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.LOW)));
        operatorController.x().onTrue(new StateManager.CmdTransitionState(superStructure, States.HOME));

        // Bumpers/Triggers
        operatorController.rightBumper().onTrue(new InstantCommand(
                () -> PickupMode.setCurrentMode(PickupMode.GROUND)));
        operatorController.leftBumper().onTrue(new InstantCommand(
                () -> PickupMode.setCurrentMode(PickupMode.STATION)));
        operatorController.leftTrigger().onTrue(new InstantCommand(
                () -> GamepieceMode.setCurrentMode(GamepieceMode.CUBE)));
        operatorController.rightTrigger().onTrue(new InstantCommand(
                () -> GamepieceMode.setCurrentMode(GamepieceMode.CONE)));
    }

    private void driverShuffleboard() {
        var tab = Shuffleboard.getTab("Driver");
        tab.addBoolean("High", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.HIGH)
            .withSize(2, 1)
            .withPosition(6, 0)
            .withProperties(Map.of("colorWhenTrue", "Blue", "colorWhenFalse", "Black"));
        tab.addBoolean("Middle", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.MID)
            .withSize(2, 1)
            .withPosition(6, 1)
            .withProperties(Map.of("colorWhenTrue", "Magenta", "colorWhenFalse", "Black"));
        tab.addBoolean("Low", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.LOW)
            .withSize(2, 1)
            .withPosition(6, 2)
            .withProperties(Map.of("colorWhenTrue", "Orange", "colorWhenFalse", "Black"));

        var pickup = tab.getLayout("Pickup Mode", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(3, 0);
        pickup.addBoolean("GROUND", () -> PickupMode.getCurrentMode() == PickupMode.GROUND);
        pickup.addBoolean("STATION", () -> PickupMode.getCurrentMode() == PickupMode.STATION);

        tab.addBoolean("Gampiece Mode", () -> GamepieceMode.getCurrentMode() == GamepieceMode.CUBE)
            .withSize(2, 2)
            .withProperties(Map.of("colorWhenTrue", "Purple", "colorWhenFalse", "Yellow"));
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
