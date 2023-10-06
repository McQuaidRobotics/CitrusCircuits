package frc.robot.commands.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GamepieceMode;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutoEventMap {
    private static HashMap<String, Command> eventMarkerMap;

    public AutoEventMap(Swerve swerve, SuperStructure superStructure) {
        eventMarkerMap = new HashMap<String, Command>() {{{
            put("placeHigh", new StateManager.CmdTransitionState(superStructure, States.PLACE_HIGH));
            put("placeMid", new StateManager.CmdTransitionState(superStructure, States.PLACE_MID));
            put("placeLow", new StateManager.CmdTransitionState(superStructure, States.PLACE_LOW));

            put("pickUpGround", new StateManager.CmdTransitionState(superStructure, States.PICKUP_GROUND));
            put("pickUpStation", new StateManager.CmdTransitionState(superStructure, States.PICKUP_STATION));

            put("standbyPosition", new StateManager.CmdTransitionState(superStructure, States.STANDBY));
            put("homePosition", new StateManager.CmdTransitionState(superStructure, States.HOME));

            put("useConeMode", new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CONE)));
            put("useCubeMode", new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CUBE)));
        }}};
    }

    public HashMap<String, Command> getMap() {
        return eventMarkerMap;
    }
}
