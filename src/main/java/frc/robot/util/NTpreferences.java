package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Robot;


public class NTpreferences {
    private static HashMap<String, Double> universalOffsets = new HashMap<String, Double>();
    private static boolean loadedCorrectly = true;

    public static enum Module{
        u3,
        u0,
        u2,
        u1
    }

    public static void loadPreferences() {
        loadedCorrectly = true;
        
        universalOffsets.put("0U", Preferences.getDouble("Module 0 Offset Universal", 0));
        universalOffsets.put("1U", Preferences.getDouble("Module 1 Offset Universal", 0));
        universalOffsets.put("2U", Preferences.getDouble("Module 2 Offset Universal", 0));
        universalOffsets.put("3U", Preferences.getDouble("Module 3 Offset Universal", 0));

        for (Double angle : universalOffsets.values()) {
            if ((angle <= 0 || angle >= 360) && Robot.isReal()) loadedCorrectly = false;
        }
    }

    public static boolean isOk() {
        return loadedCorrectly;
    }

    public static Double getRotationOffset(Module module) {
        if (module == Module.u3) return findCoterminalAngle((universalOffsets.get("3U") * 360) + 225) / 360;
        else if (module == Module.u0) return findCoterminalAngle((universalOffsets.get("0U") * 360) + 315) / 360;
        else if (module == Module.u2) return findCoterminalAngle((universalOffsets.get("2U") * 360) + 125) / 360;
        else if (module == Module.u1) return findCoterminalAngle((universalOffsets.get("1U") * 360) + 45) / 360;
        throw new IllegalArgumentException("Module not found");
    }

    public static double findCoterminalAngle(double angleOffset) {
        return (angleOffset > 360) ? angleOffset % 360 : angleOffset;
    }
}
