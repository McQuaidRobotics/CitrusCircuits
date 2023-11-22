package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
    private static Swerve swerve;

    public static void createSubsystemInstances(Swerve swerve) {
        Autos.swerve = swerve;
    }

    public enum AutoRoutines {
        NOTHING("NOTHING"),
        THREE_GAME_PIECE_FLAT("THREE_GAME_PIECE_FLAT"),
        PLACE_TAXI_WIRE("PLACE_TAXI_WIRE"),
        PLACE_BALANCE("PLACE_BALANCE");

        final String name;

        private AutoRoutines(String name) {
            this.name = name;
        }

        public Command getCommand() {
            return Commands.none();
        }
    }
}
