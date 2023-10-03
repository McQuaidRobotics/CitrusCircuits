package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Constants.*;

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
        // PathPlannerTrajectory autoPath = PathPlanner.loadPath(
        //     "Place Three Game Piece", 
        //     new PathConstraints(kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_VELOCITY));

        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> swerve.resetOdometry(autoPath.getInitialHolonomicPose())),
        //     new SwerveControllerCommand(
        //         autoPath,
        //         () -> swerve.getPose(),
        //         kSwerve.SWERVE_KINEMATICS,
        //         kAuto.AUTO_X_PID,
        //         kAuto.AUTO_Y_PID,
        //         kAuto.AUTO_ANGULAR_PID,
        //         () -> swerve.setModuleStates(),
        //         true,
        //         swerve
        //     )
        // ).withName("commandThreeGamePiece");
        return new InstantCommand().withName("commandThreeGamePiece");
    } 

    private static Command commandTwoGamePieceChargeStation(Swerve swerve, SuperStructure superStructure) {
        return new InstantCommand().withName("commandTwoGamePieceChargeStation");
    }
}
