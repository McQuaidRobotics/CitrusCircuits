package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GamepieceMode;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.commands.superstructure.OperatorPrefs.ScoreLevel;
import frc.robot.commands.superstructure.StateManager.CmdTransitionState;
import frc.robot.commands.superstructure.SuperstructureCommands.TransitionToPlace;
import frc.robot.subsystems.super_structure.States;

public class Blocks {

    public static final HashMap<String, Command> EVENT_MAP = Cmds.getMap();

    public interface Block {
        public Command getCommand(SwerveAutoBuilder builder);

        public String getCommandName();
    }

    private static String ScreamingSnakeToCamal(String screamingSnake) {
        String[] words = screamingSnake.split("_");
        String camal = "";
        for (String word : words) {
            camal += word.substring(0, 1).toUpperCase() + word.substring(1).toLowerCase();
        }
        camal = camal.substring(0, 1).toLowerCase() + camal.substring(1);
        return camal;
    }

    public static enum Cmds implements Block {
        HOME(new CmdTransitionState(RobotContainer.superStructure, States.HOME)),
        STOW(new CmdTransitionState(RobotContainer.superStructure, States.STOW)),

        PLACE(new TransitionToPlace(RobotContainer.superStructure).canFinish()),
        PLACE_STANDBY(new CmdTransitionState(RobotContainer.superStructure, States.STANDBY).canFinish()),
        PLACE_HIGH(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_HIGH).canFinish()),
        PLACE_MID(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_MID).canFinish()),
        PLACE_LOW(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_LOW_FRONT).canFinish()),

        WANT_HIGH(new InstantCommand(() -> ScoreLevel.setCurrentLevel(ScoreLevel.HIGH))),
        WANT_MID(new InstantCommand(() -> ScoreLevel.setCurrentLevel(ScoreLevel.MID))),
        WANT_LOW(new InstantCommand(() -> ScoreLevel.setCurrentLevel(ScoreLevel.LOW_FRONT))),

        PICKUP_GROUND(new CmdTransitionState(RobotContainer.superStructure, States.PICKUP_GROUND).withTimeout(1.0)),
        PICKUP_STATION(new CmdTransitionState(RobotContainer.superStructure, States.PICKUP_STATION).withTimeout(1.0)),

        DESIRE_CUBE(new InstantCommand(() -> GamepieceMode.setDesiredPiece(GamepieceMode.CUBE))),
        DESIRE_CONE(new InstantCommand(() -> GamepieceMode.setDesiredPiece(GamepieceMode.CONE))),

        OVERRIDE_HOLD_CUBE(new InstantCommand(() -> GamepieceMode.setHeldPiece(GamepieceMode.CUBE))),
        OVERRIDE_HOLD_CONE(new InstantCommand(() -> GamepieceMode.setHeldPiece(GamepieceMode.CONE))),

        DISPELL_GAMEPIECE(StateManager.dispellGamepiece(RobotContainer.superStructure));

        public final Command command;

        Cmds(Command command) {
            this.command = command;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            return command;
        }

        @Override
        public String getCommandName() {
            return "Cmd: " + ScreamingSnakeToCamal(this.name());
        }

        private static HashMap<String, Command> getMap() {
            HashMap<String, Command> map = new HashMap<String, Command>();
            for (Cmds cmd : Cmds.values()) {
                map.put(ScreamingSnakeToCamal(cmd.name()), cmd.command);
            }
            return map;
        }
    }

    public static enum Paths implements Block {
        ONE_METER(PathLoader.openFilePath("1M")),
        FLAT_BALANCE_SETUP(PathLoader.openFilePath("FLAT_BALANCE_SETUP")),
        FLAT_PICKUP3(PathLoader.openFilePath("FLAT_PICKUP3")),
        FLAT_PICKUP4(PathLoader.openFilePath("FLAT_PICKUP4")),
        FLAT_PLACE7(PathLoader.openFilePath("FLAT_PLACE7")),
        FLAT_PLACE8(PathLoader.openFilePath("FLAT_PLACE8")),
        FLAT_PLACE9(PathLoader.openFilePath("FLAT_PLACE9")),
        FLAT_SWOOP4(PathLoader.openFilePath("FLAT_SWOOP4")),
        PICKUP1_WIRE(PathLoader.openFilePath("PICKUP1_WIRE")),
        PICKUP2_WIRE(PathLoader.openFilePath("PICKUP2_WIRE")),
        PICKUP3_FLAT(PathLoader.openFilePath("PICKUP3_FLAT")),
        PICKUP4_FLAT(PathLoader.openFilePath("PICKUP4_FLAT")),
        PLACE1_WIRE(PathLoader.openFilePath("PLACE1_WIRE")),
        PLACE2_WIRE(PathLoader.openFilePath("PLACE2_WIRE")),
        PLACE7_FLAT(PathLoader.openFilePath("PLACE7_FLAT")),
        PLACE8_FLAT(PathLoader.openFilePath("PLACE8_FLAT")),
        PLACE9_FLAT(PathLoader.openFilePath("PLACE9_FLAT")),
        WIRE_BALANCE_SETUP(PathLoader.openFilePath("WIRE_BALANCE_SETUP")),
        WIRE_OVER_IN(PathLoader.openFilePath("WIRE_OVER_IN")),
        WIRE_OVER_OUT(PathLoader.openFilePath("WIRE_OVER_OUT")),
        WIRE_PICKUP1(PathLoader.openFilePath("WIRE_PICKUP1")),
        WIRE_PICKUP2(PathLoader.openFilePath("WIRE_PICKUP2")),
        WIRE_PLACE1(PathLoader.openFilePath("WIRE_PLACE1")),
        WIRE_PLACE2(PathLoader.openFilePath("WIRE_PLACE2")),
        WIRE_PLACE3(PathLoader.openFilePath("WIRE_PLACE3"));

        public final PathPlannerTrajectory traj;

        Paths(PathPlannerTrajectory traj) {
            this.traj = traj;
        }

        public PathPlannerTrajectory getTraj() {
            return traj;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            return builder.followPath(traj);
        }

        @Override
        public String getCommandName() {
            return "Path: " + ScreamingSnakeToCamal(this.name());
        }

        public CustomPath merge(Double percentThrough, Cmds block, Cmds... blocks) {
            return Blocks.merge(this.traj, percentThrough, true, block, blocks);
        }

        public CustomPath merge(Cmds block, Cmds... blocks) {
            return Blocks.merge(this.traj, 0.0, true, block, blocks);
        }
    }

    public static class CustomPath implements Block {
        private final PathPlannerTrajectory traj;

        public CustomPath(PathPlannerTrajectory traj) {
            this.traj = traj;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            return builder.followPathWithEvents(traj);
        }

        @Override
        public String getCommandName() {
            return "Custom Path";
        }

        public CustomPath merge(Double percentThrough, Cmds block, Cmds... blocks) {
            return Blocks.merge(this.traj, percentThrough, false, block, blocks);
        }

        public CustomPath merge(Cmds block, Cmds... blocks) {
            return Blocks.merge(this.traj, 0.0, false, block, blocks);
        }
    }

    private static CustomPath merge(PathPlannerTrajectory traj, Double percentThrough, Boolean eraseOld, Cmds block,
            Cmds... blocks) {
        List<String> names = new ArrayList<>();
        names.add(ScreamingSnakeToCamal(block.name()));
        for (Cmds blk : blocks) {
            names.add(ScreamingSnakeToCamal(blk.name()));
        }
        List<EventMarker> newMarkers = new ArrayList<>();
        if (!eraseOld) {
            newMarkers.addAll(traj.getMarkers());
        }
        newMarkers.add(
                EventMarker.fromTime(names, traj.getTotalTimeSeconds() * percentThrough));
        var newTraj = new PathPlannerTrajectory(
                traj.getStates(),
                newMarkers,
                new StopEvent(),
                new StopEvent(),
                false);
        return new CustomPath(newTraj);
    }

    /**
     * Merges command blocks into a path to be run at start
     * 
     * @param blocks The blocks to run at the end of the path
     * @return The merged block
     */
    public static Block sequential(Block... blocks) {
        return new Block() {
            @Override
            public Command getCommand(SwerveAutoBuilder builder) {
                List<Command> commands = new ArrayList<>();
                for (Block block : blocks) {
                    commands.add(block.getCommand(builder));
                }
                return new SequentialCommandGroup(commands.toArray(CommandBase[]::new));
            }

            @Override
            public String getCommandName() {
                return "Sequential: " + Arrays.toString(blocks);
            }
        };
    }

    public static Block[] groupBlocks(Block... blocks) {
        return blocks;
    }

    public static Command buildBlocks(Block... blocks) {
        var builder = PathLoader.getPPAutoBuilder();
        List<Command> commands = new ArrayList<>();
        var scheduler = CommandScheduler.getInstance();

        Integer count = 0;
        for (Block block : blocks) {
            var cmd = block.getCommand(builder);
            scheduler.removeComposedCommand(cmd);
            var namedCmd = cmd.withName("Block: " + count);
            commands.add(namedCmd);
            scheduler.removeComposedCommand(namedCmd);
            count++;
        }

        // commands.add(RobotContainer.swerve.commandStopDrives().withName("Stop
        // Swerve"));

        Commands.sequence(commands.toArray(CommandBase[]::new));
        return new InstantCommand();
    }
}
