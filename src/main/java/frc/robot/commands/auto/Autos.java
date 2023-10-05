package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GamepieceMode;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
    public enum AutoRoutines {
        NOTHING,
        THREE_GAME_PIECE,
        TWO_GAME_PIECE_CHARGE_STATION
    }

    public static Command getAutoRoutineCommand(AutoRoutines autoPath, Swerve swerve, SuperStructure superStructure) {
        Command autoCommand = new InstantCommand();
        switch (autoPath) {
            case THREE_GAME_PIECE: autoCommand = commandThreeGamePiece(swerve, superStructure);
            case TWO_GAME_PIECE_CHARGE_STATION: autoCommand = commandTwoGamePieceChargeStation(swerve, superStructure);
            case NOTHING:
        };
        return autoCommand;
    }

    private static Command commandThreeGamePiece(Swerve swerve, SuperStructure superStructure) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CONE)),
            new StateManager.CmdTransitionState(superStructure, States.PLACE_HIGH),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new StateManager.CmdTransitionState(superStructure, States.HOME),
                    Commands.waitSeconds(2),
                    new StateManager.CmdTransitionState(superStructure, States.PICKUP_GROUND),
                    Commands.waitSeconds(1),
                    new StateManager.CmdTransitionState(superStructure, States.STANDBY),
                    new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CUBE))
                ),
                swerve.commandRunPath("3gp pathStage1", true)
            ),

            new StateManager.CmdTransitionState(superStructure, States.PLACE_HIGH),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new StateManager.CmdTransitionState(superStructure, States.HOME),
                    new InstantCommand(() -> GamepieceMode.setCurrentMode(GamepieceMode.CONE)),
                    Commands.waitSeconds(2),
                    new StateManager.CmdTransitionState(superStructure, States.PICKUP_GROUND),
                    Commands.waitSeconds(1),
                    new StateManager.CmdTransitionState(superStructure, States.STANDBY)
                ),
                swerve.commandRunPath("3gp pathStage2", false)
            ),

            new StateManager.CmdTransitionState(superStructure, States.PLACE_LOW),
            
            new ParallelCommandGroup(
                new StateManager.CmdTransitionState(superStructure, States.HOME),
                swerve.commandRunPath("3gp pathStage3", false)
            )
        ).withName("commandThreeGamePiece");
    } 

    private static Command commandTwoGamePieceChargeStation(Swerve swerve, SuperStructure superStructure) {
        return new InstantCommand().withName("commandTwoGamePieceChargeStation");
    }
}
