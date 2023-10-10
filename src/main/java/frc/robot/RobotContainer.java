package frc.robot;

import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.commands.superstructure.OperatorPrefs.PickupMode;
import frc.robot.commands.superstructure.OperatorPrefs.ScoreLevel;
import frc.robot.commands.superstructure.OperatorPrefs;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.Autos.AutoRoutines;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    public static final Swerve swerve = new Swerve();
    public static final SuperStructure superStructure = new SuperStructure();

    public final ShuffleboardTab driverTab;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        configureDriverBindings();
        configureOperatorBindings();

        driverTab = Shuffleboard.getTab("Driver");
        configureDriverTabShuffleboard();

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> driveController.getRawAxis(translationAxis),
                        () -> driveController.getRawAxis(strafeAxis),
                        () -> -driveController.getRawAxis(rotationAxis)));
    }

    // [driver]
    // RightBumber: place position
    // LeftBumper: pickup position
    // RightTrigger: placing standby
    // LeftTrigger: stow
    // Start: zero gyro

    // [operator]
    // Y: set desired score level to high
    // B: set desired score level to mid
    // A: set desired score level to low
    // X: home/stow
    // RightBumper: set desired pickup method to ground
    // LeftBumper: set desired pickup method to shelf
    // RightTrigger: set desired gamepiece to cone
    // LeftTrigger: set desired gamepiece to cube

    private void configureDriverBindings() {
        // Bumpers/Triggers
        driveController.rightBumper().onTrue(new SuperstructureCommands.TransitionToPlace(superStructure));
        driveController.leftBumper().onTrue(new SuperstructureCommands.TransitionToPickup(superStructure));
        driveController.rightTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STANDBY));
        driveController.leftTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STOW));

        // used for testing
        // driveController.pov(0).onTrue(new InstantCommand(() ->
        // superStructure.runEndEffector(12.0, false), superStructure));
        // driveController.pov(180).onTrue(new InstantCommand(() ->
        // superStructure.runEndEffector(-12.0, false), superStructure));
        // driveController.pov(90).onTrue(new InstantCommand(() ->
        // superStructure.runEndEffector(0.0, false), superStructure));
        // driveController.x().onTrue(new
        // StateManager.CmdTransitionState(superStructure, States.STOW));
        // driveController.a().onTrue(new
        // StateManager.CmdTransitionState(superStructure, States.PLACE_LOW));
        // driveController.b().onTrue(new
        // StateManager.CmdTransitionState(superStructure, States.PLACE_MID));
        // driveController.y().onTrue(new
        // StateManager.CmdTransitionState(superStructure, States.PLACE_HIGH));

        // Center Buttons
        driveController.start().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    }

    private void configureOperatorBindings() {
        // Face buttons
        operatorController.y().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.HIGH))
                .ignoringDisable(true));
        operatorController.b().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.MID))
                .ignoringDisable(true));
        operatorController.a().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.LOW))
                .ignoringDisable(true));
        operatorController.x().onTrue(new StateManager.CmdTransitionState(superStructure, States.HOME));

        // Bumpers/Triggers
        operatorController.rightBumper().onTrue(new InstantCommand(
                () -> PickupMode.setCurrentMode(PickupMode.GROUND))
                .ignoringDisable(true));
        operatorController.leftBumper().onTrue(new InstantCommand(
                () -> PickupMode.setCurrentMode(PickupMode.STATION))
                .ignoringDisable(true));
        operatorController.leftTrigger().onTrue(new InstantCommand(
                () -> GamepieceMode.setDesiredPiece(GamepieceMode.CUBE))
                .ignoringDisable(true));
        operatorController.rightTrigger().onTrue(new InstantCommand(
                () -> GamepieceMode.setDesiredPiece(GamepieceMode.CONE))
                .ignoringDisable(true));

        // used for testing
        // operatorController.pov(0).onTrue(new InstantCommand(
        // () -> GamepieceMode.setHeldPiece(GamepieceMode.CONE))
        // .ignoringDisable(true));
        // operatorController.pov(180).onTrue(new InstantCommand(
        // () -> GamepieceMode.setHeldPiece(GamepieceMode.CUBE))
        // .ignoringDisable(true));
        // operatorController.pov(270).onTrue(new InstantCommand(
        // () -> GamepieceMode.setHeldPiece(null))
        // .ignoringDisable(true));
    }

    private void configureDriverTabShuffleboard() {
        driverTab.addBoolean("High", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.HIGH)
                .withSize(2, 1)
                .withPosition(6, 0)
                .withProperties(Map.of("colorWhenTrue", "Blue", "colorWhenFalse", "Black"));
        driverTab.addBoolean("Middle", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.MID)
                .withSize(2, 1)
                .withPosition(6, 1)
                .withProperties(Map.of("colorWhenTrue", "Magenta", "colorWhenFalse", "Black"));
        driverTab.addBoolean("Low", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.LOW)
                .withSize(2, 1)
                .withPosition(6, 2)
                .withProperties(Map.of("colorWhenTrue", "Orange", "colorWhenFalse", "Black"));

        var pickup = driverTab.getLayout("Pickup Mode", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withPosition(3, 0);
        pickup.addBoolean("GROUND", () -> PickupMode.getCurrentMode() == PickupMode.GROUND);
        pickup.addBoolean("STATION", () -> PickupMode.getCurrentMode() == PickupMode.STATION);

        driverTab.addBoolean("Desired Gamepiece", () -> GamepieceMode.getDesiredPiece() == GamepieceMode.CUBE)
                .withSize(2, 2)
                .withProperties(Map.of("colorWhenTrue", "Purple", "colorWhenFalse", "Yellow"))
                .withPosition(8, 0);

        driverTab.addString("Held Gamepiece", () -> {
            var held = GamepieceMode.getHeldPiece();
            return held == null ? "NONE" : held.toString();
        });

        driverTab.addBoolean("NEED TO HOME", () -> OperatorPrefs.NEED_HOME)
                .withPosition(3, 3)
                .withSize(2, 1)
                .withProperties(Map.of("colorWhenTrue", "Red", "colorWhenFalse", "Black"));
    }

    public Command getAutonomousCommand(AutoRoutines autoPath) {
        return Autos.getAutoRoutineCommand(autoPath, swerve, superStructure);
    }
}
