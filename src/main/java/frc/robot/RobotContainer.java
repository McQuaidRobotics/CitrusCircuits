package frc.robot;

import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.commands.superstructure.OperatorPrefs.PickupMode;
import frc.robot.commands.superstructure.OperatorPrefs.ScoreLevel;
import frc.robot.commands.superstructure.StateManager.CmdTransitionState;
import frc.robot.commands.superstructure.OperatorPrefs;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.TeleopSwerve2;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ForcibleTrigger;
import frc.robot.util.ShuffleboardApi;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private static final CommandXboxController soloController = new CommandXboxController(0);

    public static final Swerve swerve = new Swerve();
    public static final SuperStructure superStructure = new SuperStructure();

    public static void RobotContainerInit() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // configureDriverBindings();
        // configureOperatorBindings();
        configureSoloBindings();

        configureDriverTabShuffleboard();

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        soloController::getLeftY,
                        soloController::getLeftX,
                        soloController::getRightX));

    }

    private static void configureSoloBindings() {
        soloController.y().onTrue(new CmdTransitionState(RobotContainer.superStructure, States.TREBUCHET));
        soloController.b().onTrue(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_MID));
        soloController.a().onTrue(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_LOW_FRONT));

        soloController.start().onTrue(new InstantCommand(() -> swerve.setYaw(0.0)));

        soloController.rightTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STANDBY));
        soloController.leftTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STOW));

        soloController.rightBumper().onTrue(
            new InstantCommand(() -> {
                GamepieceMode.setDesiredPiece(GamepieceMode.CONE);
                GamepieceMode.setHeldPiece(GamepieceMode.CONE);
            })
                .andThen(new CmdTransitionState(superStructure, States.PICKUP_GROUND))
        );
        soloController.leftBumper().onTrue(
            new InstantCommand(() -> {
                GamepieceMode.setDesiredPiece(GamepieceMode.CUBE);
                GamepieceMode.setHeldPiece(GamepieceMode.CUBE);
            })
                .andThen(new CmdTransitionState(superStructure, States.PICKUP_GROUND))
        );

        ForcibleTrigger.from(soloController.pov(0)).onTrueForce(
                new StateManager.CmdTransitionState(superStructure, States.HOME));
    }

    private static void configureDriverTabShuffleboard() {
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

        driverTab.addBoolean("NEED TO HOME", () -> OperatorPrefs.NEED_HOME)
                .withPosition(3, 3)
                .withSize(2, 1)
                .withProperties(Map.of("colorWhenTrue", "Red", "colorWhenFalse", "Black"));
    }
}
