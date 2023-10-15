package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.Blocks.Block;
import static frc.robot.commands.auto.Blocks.Cmds;

import frc.robot.commands.auto.Blocks.PPPaths;

public class Autos {

    public enum AutoRoutines {
        NOTHING,
        THREE_GAME_PIECE,
        TWO_GAME_PIECE_WIRE,
        ONE_GAME_PIECE_TAXI
    }

    public static Command getAutoRoutineCommand(AutoRoutines autoRoutine) {
        Command autoCommand = new InstantCommand();
        switch (autoRoutine) {
            case THREE_GAME_PIECE:
                autoCommand = Blocks.buildBlocks(THREE_GAME_PIECE_FLAT);
                break;
            case TWO_GAME_PIECE_WIRE:
                autoCommand = Blocks.buildBlocks(TWO_GAME_PIECE_WIRE);
                break;
            case ONE_GAME_PIECE_TAXI:
                autoCommand = Blocks.buildBlocks(ONE_GAME_PIECE_TAXI);
                break;
            default:
                break;
        };
        return autoCommand.withName(autoRoutine + "(" + DriverStation.getAlliance() + ") Auto");
    }

    public static final Block[] THREE_GAME_PIECE_FLAT = Blocks.groupBlocks(
            Cmds.OVERRIDE_HOLD_CONE,
            Cmds.PLACE_HIGH,
            PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
            PPPaths.FLAT_SWOOP4
                    .merge(0.1, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE)
                    .merge(0.9, Cmds.STOW),
            PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY),
            Cmds.PLACE_HIGH,
            PPPaths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CONE),
            PPPaths.FLAT_PICKUP3.merge(
                    0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CONE),
            PPPaths.PICKUP3_FLAT.merge(Cmds.STOW),
            PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_STANDBY));

    public static final Block[] TWO_GAME_PIECE_WIRE = Blocks.groupBlocks(
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
            PPPaths.WIRE_OVER_OUT);

    public static final Block[] ONE_GAME_PIECE_TAXI = Blocks.groupBlocks(
            Cmds.OVERRIDE_HOLD_CONE,
            Cmds.PLACE_HIGH,
            PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.HOME));
}
