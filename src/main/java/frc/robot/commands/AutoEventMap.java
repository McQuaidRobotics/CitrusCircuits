package frc.robot.commands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GamepieceMode;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.superstructure.StateManager;

public class AutoEventMap {
    private static HashMap<String, Command> eventMarkerMap;

    public AutoEventMap(Swerve swerve, SuperStructure superStructure) {
        eventMarkerMap = new HashMap<String, Command>() {{{
            put("placeHighAuto", new StateManager.CmdTransitionState(superStructure, States.PLACE_HIGH));
            put("placeMidAuto", new StateManager.CmdTransitionState(superStructure, States.PLACE_MID));
            put("placeLowAuto", new StateManager.CmdTransitionState(superStructure, States.PLACE_LOW));
    
            put("pickUpGroundAuto", new StateManager.CmdTransitionState(superStructure, States.PICKUP_GROUND));
            put("pickUpStationAuto", new StateManager.CmdTransitionState(superStructure, States.PICKUP_STATION));

            put("standbyPositionAuto", new StateManager.CmdTransitionState(superStructure, States.STANDBY));
            put("homePositionAuto", new StateManager.CmdTransitionState(superStructure, States.HOME));
    
            put("useConeMode", new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CONE)));
            put("useCubeMode", new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CUBE)));
        }}};
    }

    public HashMap<String, Command> getEventMap() {
        return eventMarkerMap;
    }
}
