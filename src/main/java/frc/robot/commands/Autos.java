package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.super_structure.elevator.Elevator;
import frc.robot.subsystems.super_structure.pivot.Pivot;
import frc.robot.subsystems.super_structure.wrist.Wrist;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
    public enum AutoPaths {
        NOTHING,
        THREE_GAME_PIECE,
        TWO_GAME_PIECE_CHARGE_STATION
    }

    private static final HashMap<String, Command> EVENT_MARKERS = new HashMap<String, Command>() {{{
        //TODO {Maddox} Unsure where robot's cube/cone mode is in code base, but
        //need a way to switch between the modes in auto
        put("placeHighAuto", new InstantCommand());
        put("placeMidAuto", new InstantCommand());
        put("placeLowAuto", new InstantCommand());

        put("pickUpGroundAuto", new InstantCommand());
        put("pickUpStationAuto", new InstantCommand());

        put("activateRollers", new InstantCommand());
        put("deactivateRollers", new InstantCommand());

        put("useConeMode", new InstantCommand());
        put("useCubeMode", new InstantCommand());
    }}};

    public static Command getAutoPathCommand(AutoPaths autoPath, Swerve swerve, Elevator elevator, Pivot pivot, Wrist wrist) {
        Command autoCommand = new InstantCommand();
        switch (autoPath) {
            case THREE_GAME_PIECE: autoCommand = commandThreeGamePiece(swerve, elevator, pivot, wrist);
            case TWO_GAME_PIECE_CHARGE_STATION: autoCommand = commandTwoGamePieceChargeStation(swerve, elevator, pivot, wrist);
            case NOTHING:
        };
        return autoCommand;
    }

    private static Command commandThreeGamePiece(Swerve swerve, Elevator elevator, Pivot pivot, Wrist wrist) {
        return new InstantCommand().withName("commandThreeGamePiece");
    } 

    private static Command commandTwoGamePieceChargeStation(Swerve swerve, Elevator elevator, Pivot pivot, Wrist wrist) {
        return new InstantCommand().withName("commandTwoGamePieceChargeStation");
    }
}
