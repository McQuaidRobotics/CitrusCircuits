package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GamepieceMode;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.commands.superstructure.StateManager.CmdTransitionState;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutosCmdRegister {
    public static void registerCommands(Swerve swerve, SuperStructure superStructure) {
        NamedCommands.registerCommand("home", new CmdTransitionState(superStructure, States.HOME));
        NamedCommands.registerCommand("home_timeout",
            new CmdTransitionState(superStructure, States.HOME).withTimeout(0.5));
        NamedCommands.registerCommand("stow", new CmdTransitionState(superStructure, States.STOW));

        NamedCommands.registerCommand("place_standby",
            new CmdTransitionState(superStructure, States.STANDBY).canFinish());
        NamedCommands.registerCommand("place_high",
            new CmdTransitionState(superStructure, States.PLACE_HIGH).canFinish()
                .andThen(StateManager.dispellGamepiece(superStructure)));
        NamedCommands.registerCommand("place_mid",
            new CmdTransitionState(superStructure, States.PLACE_MID).canFinish()
                .andThen(StateManager.dispellGamepiece(superStructure)));
        NamedCommands.registerCommand("place_low",
            new CmdTransitionState(superStructure, States.PLACE_LOW_FRONT).canFinish()
                .andThen(StateManager.dispellGamepiece(superStructure)));
        NamedCommands.registerCommand("place_high_no_drop",
            new CmdTransitionState(superStructure, States.PLACE_HIGH).canFinish());
        NamedCommands.registerCommand("place_mid_no_drop",
            new CmdTransitionState(superStructure, States.PLACE_MID).canFinish());
        NamedCommands.registerCommand("place_low_no_drop",
            new CmdTransitionState(superStructure, States.PLACE_LOW_FRONT).canFinish());

        NamedCommands.registerCommand("pickup_ground",
            new CmdTransitionState(superStructure, States.PICKUP_GROUND));

        NamedCommands.registerCommand("pickup_station",
            new CmdTransitionState(superStructure, States.PICKUP_STATION));

        NamedCommands.registerCommand("desire_cube",
            new InstantCommand(() -> GamepieceMode.setDesiredPiece(GamepieceMode.CUBE)));
        NamedCommands.registerCommand("desire_cone",
            new InstantCommand(() -> GamepieceMode.setDesiredPiece(GamepieceMode.CONE)));

        NamedCommands.registerCommand("override_hold_cube",
            new InstantCommand(() -> GamepieceMode.setHeldPiece(GamepieceMode.CUBE)));
        NamedCommands.registerCommand("override_hold_cone",
            new InstantCommand(() -> GamepieceMode.setHeldPiece(GamepieceMode.CONE)));
    }
}
