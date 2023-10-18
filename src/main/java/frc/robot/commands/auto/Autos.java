package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.Blocks.Block;
import static frc.robot.commands.auto.Blocks.Cmds;
import static frc.robot.commands.auto.Blocks.PPPaths;
import static frc.robot.commands.auto.Blocks.AlliancePath;;

public class Autos {

    public static final Block[] THREE_GAME_PIECE_FLAT = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_STANDBY,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.FLAT_SWOOP4
                .merge(0.1, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE)
                .merge(0.8, Cmds.STOW),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY),
        Cmds.PLACE_HIGH,
        PPPaths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CONE),
        PPPaths.FLAT_PICKUP3.merge(
                0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CONE),
        PPPaths.PICKUP3_FLAT.merge(Cmds.STOW),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY));

    public static final Block[] THREE_GAME_PIECE_FLAT_ALT = Blocks.groupBlocks(
            Cmds.OVERRIDE_HOLD_CONE,
            Cmds.PLACE_STANDBY,
            Cmds.PLACE_HIGH,
            PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
            new AlliancePath(PPPaths.FLAT_SWOOP4B_BLUE, PPPaths.FLAT_SWOOP4B_RED)
                    .merge(0.1, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE)
                    .merge(0.8, Cmds.STOW),
            PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY),
            Cmds.PLACE_HIGH,
            PPPaths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CONE),
            PPPaths.FLAT_PICKUP3.merge(
                    0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CONE),
            PPPaths.PICKUP3_FLAT.merge(Cmds.STOW),
            PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY));

    public static final Block[] TWO_HALF_GAME_PIECE_WIRE = Blocks.groupBlocks(
            Cmds.OVERRIDE_HOLD_CONE,
            Cmds.PLACE_HIGH,
            PPPaths.PLACE1_WIRE.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
            PPPaths.WIRE_OVER_OUT,
            PPPaths.WIRE_PICKUP1.merge(0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
            PPPaths.PICKUP1_WIRE.merge(Cmds.STOW),
            PPPaths.WIRE_OVER_IN,
            PPPaths.WIRE_PLACE2.merge(0.2, Cmds.PLACE_STANDBY),
            Cmds.PLACE_HIGH,
            PPPaths.PLACE2_WIRE.merge(Cmds.STOW),
            PPPaths.WIRE_OVER_OUT,
            PPPaths.WIRE_PICKUP2.merge(0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
            Cmds.STOW);

    public static final Block[] ONE_GAME_PIECE_TAXI = Blocks.groupBlocks(
            Cmds.OVERRIDE_HOLD_CONE,
            Cmds.PLACE_HIGH,
            PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.HOME));

    public enum AutoRoutines {
        NOTHING(new Block[] {}),
        THREE_GAME_PIECE(Autos.THREE_GAME_PIECE_FLAT),
        THREE_GAME_PIECE_ALT(Autos.THREE_GAME_PIECE_FLAT_ALT),
        TWO_HALF_GAME_PIECE_WIRE(Autos.TWO_HALF_GAME_PIECE_WIRE),
        ONE_GAME_PIECE_TAXI(Autos.ONE_GAME_PIECE_TAXI),
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
