package frc.robot;

public enum GamepieceMode {
    CONE, CUBE;

    private static GamepieceMode currentMode = CONE;

    public static GamepieceMode getCurrentMode() {
        return currentMode;
    }

    public static void setCurrentMode(GamepieceMode mode) {
        currentMode = mode;
    }
}
