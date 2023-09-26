package frc.robot.commands.superstructure;

public class OperatorPrefs {

    public static enum PickupMode {
        GROUND,
        STATION;

        private static PickupMode currentMode = GROUND;

        public synchronized static void setCurrentMode(PickupMode mode) {
            currentMode = mode;
        }

        public synchronized static PickupMode getCurrentMode() {
            return currentMode;
        }
    }

    public static enum ScoreLevel {
        LOW,
        MIDDLE,
        HIGH;

        private static ScoreLevel currentLevel = HIGH;

        public synchronized static void setCurrentLevel(ScoreLevel level) {
            currentLevel = level;
        }

        public synchronized static ScoreLevel getCurrentLevel() {
            return currentLevel;
        }
    }


}
