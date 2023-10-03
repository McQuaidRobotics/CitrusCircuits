package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
    public enum AutoPaths {
        NOTHING,
        THREE_GAME_PIECE,
        TWO_GAME_PIECE_CHARGE_STATION
    }

    public static Command getAutoPathCommand(AutoPaths autoPath, Swerve swerve, SuperStructure superStructure) {
        Command autoCommand = new InstantCommand();
        switch (autoPath) {
            case THREE_GAME_PIECE: autoCommand = commandThreeGamePiece(swerve, superStructure);
            case TWO_GAME_PIECE_CHARGE_STATION: autoCommand = commandTwoGamePieceChargeStation(swerve, superStructure);
            case NOTHING:
        };
        return autoCommand;
    }

    private static Command commandThreeGamePiece(Swerve swerve, SuperStructure superStructure) {
        return new InstantCommand().withName("commandThreeGamePiece");
    } 

    private static Command commandTwoGamePieceChargeStation(Swerve swerve, SuperStructure superStructure) {
        return new InstantCommand().withName("commandTwoGamePieceChargeStation");
    }
}
