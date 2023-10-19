package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.Blocks.Block;
import static frc.robot.commands.auto.Blocks.Cmds;
import static frc.robot.commands.auto.Blocks.PPPaths;

public class Autos {

    public static final Block[] THREE_GAME_PIECE_FLAT_CONE = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_STANDBY,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.FLAT_SWOOP4B
            .merge(0.1, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE)
            .merge(0.7, Cmds.STOW),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY),
        Cmds.PLACE_HIGH,
        PPPaths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CONE),
        PPPaths.FLAT_PICKUP3.merge(
            0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CONE),
        PPPaths.PICKUP3_FLAT.merge(Cmds.STOW),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY)
    );

    public static final Block[] THREE_GAME_PIECE_FLAT_CUBE = Blocks.groupBlocks(
        Cmds.SET_LAST_STATE_STARTUP,
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_MID,
        PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.FLAT_SWOOP4B
            .merge(0.1, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE)
            .merge(0.65, Cmds.PLACE_STANDBY),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_HIGH_NO_DROP),
        Cmds.PLACE_HIGH,
        PPPaths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.FLAT_PICKUP3.merge(
            0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
        PPPaths.PICKUP3_FLAT.merge(Cmds.STOW).merge(0.7, Cmds.PLACE_STANDBY),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_MID),
        // Cmds.PLACE_MID,
        Cmds.STOW
    );

    public static final Block[] TWO_HALF_GAME_PIECE_WIRE = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE1_WIRE.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.WIRE_OVER_OUT,
        PPPaths.WIRE_PICKUP1.merge(0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
        PPPaths.PICKUP1_WIRE.merge(Cmds.STOW),
        PPPaths.WIRE_OVER_IN,
        PPPaths.WIRE_PLACE2.merge(0.2, Cmds.PLACE_STANDBY),
        Cmds.PLACE_LOW,
        PPPaths.PLACE2_WIRE.merge(Cmds.STOW),
        PPPaths.WIRE_OVER_OUT,
        PPPaths.WIRE_PICKUP2.merge(0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
        Cmds.STOW
    );

    public static final Block[] POINT_FIVE_GAME_PIECE_WIRE = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE1_WIRE.resetPose(),
        PPPaths.WIRE_OVER_OUT,
        PPPaths.WIRE_PICKUP1
    );

    public static final Block[] ONE_GAME_PIECE_TAXI = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW)
    );

    public enum AutoRoutines {
        NOTHING(new Block[] {}),
        THREE_GAME_PIECE_CONE(Autos.THREE_GAME_PIECE_FLAT_CONE),
        THREE_GAME_PIECE_FLAT_CUBE(Autos.THREE_GAME_PIECE_FLAT_CUBE),
        TWO_HALF_GAME_PIECE_WIRE(Autos.TWO_HALF_GAME_PIECE_WIRE),
        ONE_GAME_PIECE_TAXI(Autos.ONE_GAME_PIECE_TAXI),
        POINT_FIVE_GAME_PIECE_WIRE(Autos.POINT_FIVE_GAME_PIECE_WIRE),
        STRAIGHT_LINE(PPPaths.ONE_METER.resetPose());

        private final Block[] blocks;

        private AutoRoutines(Block... blocks) {
            this.blocks = blocks;
        }

        public Command getCommand() {
            return Blocks.buildBlocks(blocks).withName(this + "(" + DriverStation.getAlliance() + ") Auto");
        }
    }
}
