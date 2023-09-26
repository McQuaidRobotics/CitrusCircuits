package frc.robot;

public enum GamepieceMode {
    CONE, CUBE;

    private static GamepieceMode currentMode = CONE;

    public synchronized static GamepieceMode getCurrentMode() {
        return currentMode;
    }

    public synchronized static void setCurrentMode(GamepieceMode mode) {
        currentMode = mode;
    }
}
