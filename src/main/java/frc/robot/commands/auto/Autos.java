package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.Blocks.Block;
import frc.robot.commands.auto.Blocks.Cmds;
import frc.robot.commands.auto.Blocks.Paths;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {

    public enum AutoRoutines {
        NOTHING,
        THREE_GAME_PIECE,
        TWO_GAME_PIECE_CHARGE_STATION
    }

    public static Command getAutoRoutineCommand(AutoRoutines autoRoutine, Swerve swerve, SuperStructure superStructure) {
        Command autoCommand = new InstantCommand();
        switch (autoRoutine) {
            // case THREE_GAME_PIECE: autoCommand = commandThreeGamePiece(swerve, superStructure);
            // case TWO_GAME_PIECE_CHARGE_STATION: autoCommand = commandTwoGamePieceChargeStation(swerve, superStructure);
            case NOTHING:
        };
        return autoCommand;
    }

    public static final Block[] THREE_GAME_PIECE = Blocks.groupBlocks(
        Cmds.PLACE_HIGH,
        Paths.PLACE9_FLAT.merge(Cmds.HOME, Cmds.DESIRE_CUBE),
        // Paths.FLAT_PICKUP4.merge(0.8, Cmds.PICKUP_GROUND),
        // Paths.PICKUP4_FLAT.merge(Cmds.STOW),
        Paths.FLAT_SWOOP4
            .merge(0.45, Cmds.PICKUP_GROUND)
            .merge(0.6, Cmds.STOW),
        Paths.FLAT_PLACE8,
        Cmds.PLACE_HIGH,
        Paths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CONE),
        Paths.FLAT_PICKUP3.merge(0.8, Cmds.PICKUP_GROUND),
        Paths.PICKUP3_FLAT.merge(Cmds.STOW),
        Paths.FLAT_PLACE7,
        Cmds.PLACE_HIGH,
        Paths.PLACE7_FLAT.merge(Cmds.STOW)
    );
}
