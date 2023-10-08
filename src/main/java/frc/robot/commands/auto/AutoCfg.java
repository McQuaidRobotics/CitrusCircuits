package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GamepieceMode;

public class AutoCfg {

    public enum StartPositions {
        LEFT(0.0, 0.0),
        CENTER(0.0, 0.0),
        RIGHT(0.0, 0.0);

        public final double x;
        public final double y;

        private StartPositions(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public final GamepieceMode[] gamepieces;
    public final StartPositions startPosition;
    public final Alliance alliance;
    public final Boolean balance;

    public AutoCfg(GamepieceMode[] gamepieces, StartPositions startPosition, Alliance alliance, Boolean balance) {
        this.gamepieces = gamepieces;
        this.startPosition = startPosition;
        this.alliance = alliance;
        this.balance = balance;
    }
}
