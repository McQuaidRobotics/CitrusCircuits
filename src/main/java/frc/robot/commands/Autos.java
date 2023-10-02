package frc.robot.commands;

import javax.lang.model.element.ElementVisitor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public static Command getAutoPathCommand(AutoPaths autoPath, Swerve swerve, Elevator elevator, Pivot pivot, Wrist wrist) {
        Command autoCommand = new InstantCommand();
        switch (autoPath) {
            case THREE_GAME_PIECE: autoCommand = commandThreeGamePiece(swerve, elevator, pivot, wrist);
            case TWO_GAME_PIECE_CHARGE_STATION: autoCommand = commandTwoGamePieceChargeStation(swerve, elevator, pivot, wrist);
            case NOTHING: autoCommand = new InstantCommand();
        };
        return autoCommand;
    }

    public static Command commandThreeGamePiece(Swerve swerve, Elevator elevator, Pivot pivot, Wrist wrist) {
        return new InstantCommand().withName("commandThreeGamePiece");
    } 

    public static Command commandTwoGamePieceChargeStation(Swerve swerve, Elevator elevator, Pivot pivot, Wrist wrist) {
        return new InstantCommand().withName("commandTwoGamePieceChargeStation");
    }
}
