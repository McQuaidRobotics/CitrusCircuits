package frc.robot;

import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.commands.superstructure.OperatorPrefs.PickupMode;
import frc.robot.commands.superstructure.OperatorPrefs.ScoreLevel;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.auto.AutosCmdRegister;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ForcibleTrigger;
import frc.robot.util.ShuffleboardApi;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final Swerve swerve = new Swerve();
    private final SuperStructure superStructure = new SuperStructure();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        setupAutos();

        configureDriverBindings();
        configureOperatorBindings();

        configureDriverTabShuffleboard();

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                driveController::getLeftY,
                driveController::getLeftX,
                driveController::getRightX
            )
        );
    }

    // [driver]
    // RightBumber: place position
    // LeftBumper: pickup position
    // RightTrigger: placing standby
    // LeftTrigger: stow
    // D-pad UP: home/stow
    // Start: zero gyro

    // [operator]
    // Y: set desired score level to high
    // B: set desired score level to mid
    // A: set desired score level to low front
    // X: set desired score level to low back
    // D-pad UP: home/stow
    // RightBumper: set desired pickup method to ground
    // LeftBumper: set desired pickup method to shelf
    // RightTrigger: set desired gamepiece to cone
    // LeftTrigger: set desired gamepiece to cube

    private void configureDriverBindings() {
        // Bumpers/Triggers
        ForcibleTrigger.from(driveController.rightBumper())
                .onTrueForce(new SuperstructureCommands.TransitionToPlace(superStructure));
        ForcibleTrigger.from(driveController.leftBumper())
                .onTrueForce(new SuperstructureCommands.TransitionToPickup(superStructure));
        driveController.rightTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STANDBY));
        driveController.leftTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STOW));

        // Center Buttons
        driveController.start().onTrue(new InstantCommand(() -> swerve.setYaw(0.0)));

        // POV buttons
        ForcibleTrigger.from(driveController.pov(0))
                .onTrueForce(new StateManager.CmdTransitionState(superStructure, States.HOME));
    }

    private void configureOperatorBindings() {
        // Face buttons
        operatorController.y().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.HIGH)).ignoringDisable(true));
        operatorController.b().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.MID)).ignoringDisable(true));
        operatorController.a().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.LOW_FRONT)).ignoringDisable(true));
        operatorController.x().onTrue(new InstantCommand(
                () -> ScoreLevel.setCurrentLevel(ScoreLevel.LOW_BACK)).ignoringDisable(true));

        // Bumpers/Triggers
        operatorController.rightBumper().onTrue(new InstantCommand(
                () -> PickupMode.setCurrentMode(PickupMode.GROUND)).ignoringDisable(true));
        operatorController.leftBumper().onTrue(new InstantCommand(
                () -> PickupMode.setCurrentMode(PickupMode.STATION)).ignoringDisable(true));
        operatorController.leftTrigger().onTrue(new InstantCommand(
                () -> GamepieceMode.setDesiredPiece(GamepieceMode.CUBE)).ignoringDisable(true));
        operatorController.rightTrigger().onTrue(new InstantCommand(
                () -> GamepieceMode.setDesiredPiece(GamepieceMode.CONE)).ignoringDisable(true));

        // POV buttons
        ForcibleTrigger.from(operatorController.pov(0)).onTrueForce(
                new StateManager.CmdTransitionState(superStructure, States.HOME));
    }

    private void configureDriverTabShuffleboard() {
        var driverTab = ShuffleboardApi.getTab("Driver");
        driverTab.addBoolean("High", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.HIGH)
                .withSize(2, 1)
                .withPosition(6, 0)
                .withProperties(Map.of("colorWhenTrue", "Blue", "colorWhenFalse", "Black"));
        driverTab.addBoolean("Middle", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.MID)
                .withSize(2, 1)
                .withPosition(6, 1)
                .withProperties(Map.of("colorWhenTrue", "Magenta", "colorWhenFalse", "Black"));
        driverTab.addBoolean("Low Front", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.LOW_FRONT)
                .withSize(2, 1)
                .withPosition(6, 2)
                .withProperties(Map.of("colorWhenTrue", "Orange", "colorWhenFalse", "Black"));
        driverTab.addBoolean("Low Back", () -> ScoreLevel.getCurrentLevel() == ScoreLevel.LOW_BACK)
                .withSize(2, 1)
                .withPosition(6, 3)
                .withProperties(Map.of("colorWhenTrue", "Orange", "colorWhenFalse", "Black"));

        driverTab.addBoolean("GROUND", () -> PickupMode.getCurrentMode() == PickupMode.GROUND)
                .withSize(2, 1)
                .withPosition(3, 0);
        driverTab.addBoolean("STATION", () -> PickupMode.getCurrentMode() == PickupMode.STATION)
                .withSize(2, 1)
                .withPosition(3, 1);

        driverTab.addBoolean("Desired Gamepiece", () -> GamepieceMode.getDesiredPiece() == GamepieceMode.CUBE)
                .withSize(2, 2)
                .withProperties(Map.of("colorWhenTrue", "Purple", "colorWhenFalse", "Yellow"))
                .withPosition(8, 0);

        driverTab.addString("Held Gamepiece", () -> {
            var held = GamepieceMode.getHeldPiece();
            return held == null ? "NONE" : held.toString();
        }).withSize(2, 1);
    }

    private void setupAutos() {
        AutosCmdRegister.registerCommands(swerve, superStructure);
        AutoBuilder.configureHolonomic(
                swerve::getPose, 
                swerve::resetOdometry, 
                swerve::getChassisSpeeds, 
                swerve::driveRobotRelative, 
                new HolonomicPathFollowerConfig(
                        kAuto.AUTO_TRANSLATION_PID,
                        kAuto.AUTO_ANGULAR_PID,
                        kSwerve.MAX_SPEED,
                        kSwerve.DRIVEBASE_RADIUS,
                        new ReplanningConfig(
                                true, 
                                false)
                ),
                swerve);
    }
}
